import uk.co.transputersystems.transputer.simulator.{Transputer, TransputerConstants}

import java.io.{File, FileWriter, PrintWriter}
import java.util
import java.util.{HashSet, List, Scanner}
import scala.sys.process.stdout

object Main extends App {
run(Array.empty[String])

  @throws[Exception]
  def run(args: Array[String]): Unit = {
    val files = Array[File]( new File("hello.btl"))

    val config = SimulatorConfig(false,  null,  null,  null,   files, false)
    var transputers: Array[Transputer] = null
    var activeTransputers: util.HashSet[Transputer] = null
    var anyTransputerActive = true
    var hitBreak = false
    var currentlyInteractive = config.interactive
    var loopCount = 0
    val stdout = new PrintWriter(System.out)
    val stderr = new PrintWriter(System.err)
    val stdin = new Scanner(System.in)
    var testCheckerFileWriter: FileWriter = null
    var testCheckerPrintWriter: PrintWriter = null
    var schedCheckerFileWriter: FileWriter = null
    var schedCheckerPrintWriter: PrintWriter = null
    var timerCheckerFileWriter: FileWriter = null
    var timerCheckerPrintWriter: PrintWriter = null
    //int i, j;
    var worked = false
    if (config.testChecker != null) {
      // Open output file and write initial state
      testCheckerFileWriter = new FileWriter(config.testChecker, false)
      testCheckerPrintWriter = new PrintWriter(testCheckerFileWriter)
    }
    if (config.schedChecker != null) {
      // Open output file and write initial state
      schedCheckerFileWriter = new FileWriter(config.schedChecker, false)
      schedCheckerPrintWriter = new PrintWriter(schedCheckerFileWriter)
    }
    if (config.timerChecker != null) {
      timerCheckerFileWriter = new FileWriter(config.timerChecker, false)
      timerCheckerPrintWriter = new PrintWriter(timerCheckerFileWriter)
    }
    stdout.printf("# Loading\n")
    transputers = new Array[Transputer](config.binaries.size)
    activeTransputers = new util.HashSet[Transputer]
    for (i <- 0 until config.binaries.size) {
      transputers(i) = new Transputer(i.toByte, stdout, stderr)
      transputers(i).loadProgram(config.binaries(i))
      transputers(i).printRecentMemory(stdout)
      transputers(i).printRegisters(stdout)
      if (config.testChecker != null) transputers(i).logState(0, testCheckerPrintWriter)
      if (config.schedChecker != null) transputers(i).logSched(0, schedCheckerPrintWriter)
      if (config.timerChecker != null) transputers(i).logTimer(0, timerCheckerPrintWriter)
      activeTransputers.add(transputers(i))
    }
    stdout.printf("# Starting\n")
    stdout.flush()
    stderr.flush()
    while (anyTransputerActive) {
      // Check if we hit any breakpoint
      hitBreak = false
      for (transputer <- transputers) {
        if (activeTransputers.contains(transputer)) hitBreak = hitBreak || transputer.debuggerState.breakpoints.contains(transputer.registers.Iptr)
      }
//      if (currentlyInteractive || hitBreak) currentlyInteractive = interact(transputers, stdin, stdout, stderr)
      anyTransputerActive = false
      loopCount += 1
      for (transputer <- transputers) {
        if (activeTransputers.contains(transputer)) {
          worked = transputer.performStep
          transputer.printRegisters(stdout)
          transputer.incrementClock(loopCount)
          if (transputer.programEndPtr < transputer.registers.Iptr || transputer.registers.Iptr < TransputerConstants.CODESTART) activeTransputers.remove(transputer)
          else {
            anyTransputerActive = true
            // Check LinkIn
            var j = 0
            while (j < TransputerConstants.IN_PORTS) {
              transputer.processInputLink(transputer.inputLinks(j))

              j += 1
            }
            // Check LinkOut
            transputer.processOutputLink()
          }
          if (config.testChecker != null) transputer.logState(loopCount - 1, testCheckerPrintWriter)
          if (config.schedChecker != null) transputer.logSched(loopCount - 1, schedCheckerPrintWriter)
          if (worked && config.timerChecker != null) transputer.logTimer(loopCount - 1, timerCheckerPrintWriter)
          if (!worked) activeTransputers.remove(transputer)
        }
      }
      Transputer.switchStep(transputers, stdout)
      stdout.flush()
      stderr.flush()
    }
    for (transputer <- transputers) {
      transputer.printRegisters(stdout)
      transputer.printRecentMemory(stdout)
    }
    if (config.testChecker != null) {
      testCheckerFileWriter.close()
      stdout.printf("# Closed log file for testing\n")
    }
    if (config.schedChecker != null) {
      schedCheckerFileWriter.close()
      stdout.printf("# Closed log file for scheduler checking\n")
    }
    if (config.timerChecker != null) {
      timerCheckerFileWriter.close()
      stdout.printf("# Closed log file for timer checking\n")
    }
    if (config.printWorkspaceMemory) {
      stdout.printf("# Workspace memory usage\n")
      for (transputer <- transputers) {
        transputer.printWorkspaceMemory(stdout)
      }
    }
    stdout.println()
    stdout.printf("# Total steps: %d\n", loopCount)
    stdout.printf("\n==DONE==\n")
    stdout.flush()
    stderr.flush()
  }

//  private def interact(transputers: Array[Transputer], input: Scanner, output: PrintWriter, errOutput: PrintWriter): Boolean = {
//    var command: String = null
//    var result : CommandResult.Value = null
//    while (true) {
//      System.out.printf("> ")
//      command = input.nextLine
//      output.printf("\n")
//      executeCommand(command, transputers, output, errOutput) match {
//        case CommandResult.CONTINUE =>
//          return false
//        case CommandResult. STEP =>
//          return true
//        case _ =>
//      }
//    }
//  }
}


case class SimulatorConfig(val interactive: Boolean,  val testChecker: File,  val schedChecker: File,  val timerChecker: File,  val binaries: Array[File], val printWorkspaceMemory: Boolean)


object CommandResult extends Enumeration {
  type CommandResult = Value
  val REMAIN, // Do not step to the next instruction
  STEP, // Step to the next instruction
  CONTINUE, // Continue to execute without stopping
  NOT_RECOGNISED // Command was not recognised
  = Value
}
