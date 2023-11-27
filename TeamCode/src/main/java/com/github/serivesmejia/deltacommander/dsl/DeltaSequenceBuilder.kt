package com.github.serivesmejia.deltacommander.dsl

import com.github.serivesmejia.deltacommander.DeltaCommand
import com.github.serivesmejia.deltacommander.DeltaSubsystem
import com.github.serivesmejia.deltacommander.command.DeltaInstantCmd
import com.github.serivesmejia.deltacommander.command.DeltaSequentialCmd
import com.github.serivesmejia.deltacommander.command.DeltaWaitCmd
import com.github.serivesmejia.deltacommander.command.DeltaWaitConditionCmd
import com.github.serivesmejia.deltacommander.deltaScheduler
import java.lang.IllegalArgumentException
import kotlin.reflect.KClass

class DeltaSequenceBuilder(private val block: DeltaSequenceBuilder.() -> Unit) {

    private val commands = mutableListOf<DeltaCommand>()

    private val reqs = mutableListOf<DeltaSubsystem>()

    operator fun <T : DeltaCommand> T.unaryMinus(): T {
        commands.add(this)
        return this
    }

    fun DeltaCommand.async() = DeltaInstantCmd(this::schedule)

    fun waitFor(condition: () -> Boolean) = DeltaWaitConditionCmd(condition)

    fun waitForSeconds(seconds: Double) = DeltaWaitCmd(seconds)

    fun require(subsystem: DeltaSubsystem) = reqs.add(subsystem)

    inline fun <reified S : DeltaSubsystem> require() = require(S::class)

    @Suppress("UNCHECKED_CAST")
    fun <S : DeltaSubsystem> require(clazz: KClass<S>): S {
        for(subsystem in deltaScheduler.subsystems) {
            if(subsystem::class == clazz) {
                require(subsystem)
                return subsystem as S
            }
        }

        throw IllegalArgumentException("Unable to find subsystem ${clazz::class.java.name} in DeltaScheduler")
    }

    @JvmName("requireByName")
    inline fun <reified S : DeltaSubsystem> require(clazzName: String): S {
        val sub = require(clazzName)

        if(sub is S)
            return sub
        else throw IllegalArgumentException("Unable to find subsystem $clazzName with type ${S::class.java.name} in DeltaScheduler")
    }

    fun require(clazzName: String): DeltaSubsystem {
        for(subsystem in deltaScheduler.subsystems) {
            if(subsystem::class.java.simpleName == clazzName) {
                require(subsystem)
                return subsystem
            }
        }

        throw IllegalArgumentException("Unable to find subsystem $clazzName in DeltaScheduler")
    }

    inline fun <reified C: DeltaCommand> C.stopOn(noinline condition: C.() -> Boolean): DeltaCommand {
        val command = this

        - deltaSequence {
            - waitFor { command.hasRunOnce && condition(command) }
            - DeltaInstantCmd(command::requestEnd)
        }.async()

        return this
    }

    inline fun <reified C: DeltaCommand> C.waitUntil(noinline condition: C.() -> Boolean): DeltaCommand {
        val command = this
        return deltaSequence {
            - command.async()
            - waitFor { command.hasRunOnce && condition(command) }
        }
    }

    internal fun build(): DeltaSequentialCmd {
        block()
        return DeltaSequentialCmd(*commands.toTypedArray()).apply {
            require(*(reqs).toTypedArray())
        }
    }

}

fun deltaSequence(block: DeltaSequenceBuilder.() -> Unit) = DeltaSequenceBuilder(block).build()

fun deltaSequenceInstant(block: DeltaSequenceBuilder.() -> Unit) = DeltaInstantCmd {
    + deltaSequence(block)
}