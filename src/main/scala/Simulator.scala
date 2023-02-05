//package uk.co.transputersystems.transputer.simulator
//
//import java.io.File
//import java.io.FileWriter
//import java.io.IOException
//import java.io.PrintWriter
//import java.util
//import java.util.Scanner
//import java.io.File.pathSeparatorChar
//import joptsimple.OptionParser
//import joptsimple.OptionSet
//import joptsimple.OptionSpec
//import org.antlr.v4.runtime.ANTLRInputStream
//import org.antlr.v4.runtime.CommonTokenStream
//import org.antlr.v4.runtime.ConsoleErrorListener
//import org.antlr.v4.runtime.tree.ParseTree
//object Simulator {
//  private def executeCommand(command: String, transputers: Array[Transputer], output: PrintWriter, errOutput: PrintWriter): CommandResult = {
//    val commandLexer = new Nothing(new ANTLRInputStream(command))
//    commandLexer.removeErrorListener(ConsoleErrorListener.INSTANCE)
//    val tokenStream = new CommonTokenStream(commandLexer)
//    val errorListener = new ErrorListener
//    val commandParser = new Nothing(tokenStream)
//    commandParser.addErrorListener(errorListener)
//    commandParser.removeErrorListener(ConsoleErrorListener.INSTANCE)
//    val commandTree = commandParser.command
//    if (errorListener.errors != 0) {
//      output.println("Command not recognised.")
//      output.flush()
//      return CommandResult.NOT_RECOGNISED
//    }
//    val executor = new CommandExecutor(transputers, output, errOutput)
//    executor.visit(commandTree)
//  }
//
//  private def interact(transputers: Array[Transputer], input: Scanner, output: PrintWriter, errOutput: PrintWriter): Boolean = {
//    var command: String = null
//    val result: CommandResult = null
//    while (true) {
//      System.out.printf("> ")
//      command = input.nextLine
//      output.printf("\n")
//      executeCommand(command, transputers, output, errOutput) match {
//        case CONTINUE =>
//          return false
//        case STEP =>
//          return true
//        case _ =>
//      }
//    }
//  }
//
//  def parseOptions(args: Array[String]): SimulatorConfig = {
//    val optionParser = new OptionParser
//    val interactiveArg = optionParser.accepts("interactive")
//    val printWorkspaceMemArg = optionParser.accepts("print-workspace-mem")
//    val verilogTestbenchArg = optionParser.accepts("verilog-testbench-gen").withRequiredArg.ofType(classOf[File]).describedAs("generate test checking files for verilog testbench")
//    val schedulerArg = optionParser.accepts("scheduler-test").withRequiredArg.ofType(classOf[File]).describedAs("generate test checking including scheduler registers")
//    val timerArg = optionParser.accepts("timer-test").withRequiredArg.ofType(classOf[File]).describedAs("generate test checking including timer registers")
//    val binariesArg = optionParser.accepts("binaries").withRequiredArg.required.ofType(classOf[File]).withValuesSeparatedBy(pathSeparatorChar)
//    val options = optionParser.parse(args)
//    val config = new SimulatorConfig(options.has(interactiveArg), options.valueOf(verilogTestbenchArg), options.valueOf(schedulerArg), options.valueOf(timerArg), options.valuesOf(binariesArg), options.has(printWorkspaceMemArg))
//    if (config.binaries.size == 0) throw new IllegalArgumentException("At least one binary must be supplied.")
//    else if (config.binaries.size > 4) {
//      // TODO: lift this restriction?
//      throw new IllegalArgumentException("At most four binaries can be supplied.")
//    }
//    config
//  }
//
//  @throws[Exception]
//  def run(args: Array[String]): Unit = {
//    val config = parseOptions(args)
//    var transputers: Array[Transputer] = null
//    var activeTransputers: HashSet[Transputer] = null
//    var anyTransputerActive = true
//    var hitBreak = false
//    var currentlyInteractive = config.interactive
//    var loopCount = 0
//    val stdout = new PrintWriter(System.out)
//    val stderr = new PrintWriter(System.err)
//    val stdin = new Scanner(System.in)
//    var testCheckerFileWriter: FileWriter = null
//    var testCheckerPrintWriter: PrintWriter = null
//    var schedCheckerFileWriter: FileWriter = null
//    var schedCheckerPrintWriter: PrintWriter = null
//    var timerCheckerFileWriter: FileWriter = null
//    var timerCheckerPrintWriter: PrintWriter = null
//    //int i, j;
//    var worked = false
//    if (config.testChecker != null) {
//      // Open output file and write initial state
//      testCheckerFileWriter = new FileWriter(config.testChecker, false)
//      testCheckerPrintWriter = new PrintWriter(testCheckerFileWriter)
//    }
//    if (config.schedChecker != null) {
//      // Open output file and write initial state
//      schedCheckerFileWriter = new FileWriter(config.schedChecker, false)
//      schedCheckerPrintWriter = new PrintWriter(schedCheckerFileWriter)
//    }
//    if (config.timerChecker != null) {
//      timerCheckerFileWriter = new FileWriter(config.timerChecker, false)
//      timerCheckerPrintWriter = new PrintWriter(timerCheckerFileWriter)
//    }
//    stdout.printf("# Loading\n")
//    transputers = new Array[Transputer](config.binaries.size)
//    activeTransputers = new HashSet[Transputer]
//    for (i <- 0 until config.binaries.size) {
//      transputers(i) = new Transputer(i.toByte, stdout, stderr)
//      transputers(i).loadProgram(config.binaries.get(i))
//      transputers(i).printRecentMemory(stdout)
//      transputers(i).printRegisters(stdout)
//      if (config.testChecker != null) transputers(i).logState(0, testCheckerPrintWriter)
//      if (config.schedChecker != null) transputers(i).logSched(0, schedCheckerPrintWriter)
//      if (config.timerChecker != null) transputers(i).logTimer(0, timerCheckerPrintWriter)
//      activeTransputers.add(transputers(i))
//    }
//    stdout.printf("# Starting\n")
//    stdout.flush()
//    stderr.flush()
//    while (anyTransputerActive) {
//      // Check if we hit any breakpoint
//      hitBreak = false
//      for (transputer <- transputers) {
//        if (activeTransputers.contains(transputer)) hitBreak = hitBreak || transputer.debuggerState.breakpoints.contains(transputer.registers.Iptr)
//      }
//      if (currentlyInteractive || hitBreak) currentlyInteractive = interact(transputers, stdin, stdout, stderr)
//      anyTransputerActive = false
//      loopCount += 1
//      for (transputer <- transputers) {
//        if (activeTransputers.contains(transputer)) {
//          worked = transputer.performStep
//          transputer.printRegisters(stdout)
//          transputer.incrementClock(loopCount)
//          if (transputer.programEndPtr < transputer.registers.Iptr || transputer.registers.Iptr < TransputerConstants.CODESTART) activeTransputers.remove(transputer)
//          else {
//            anyTransputerActive = true
//            // Check LinkIn
//            var j = 0
//            while (j < TransputerConstants.IN_PORTS) {
//              transputer.processInputLink(transputer.inputLinks(j))
//
//              j += 1
//            }
//            // Check LinkOut
//            transputer.processOutputLink()
//          }
//          if (config.testChecker != null) transputer.logState(loopCount - 1, testCheckerPrintWriter)
//          if (config.schedChecker != null) transputer.logSched(loopCount - 1, schedCheckerPrintWriter)
//          if (worked && config.timerChecker != null) transputer.logTimer(loopCount - 1, timerCheckerPrintWriter)
//          if (!worked) activeTransputers.remove(transputer)
//        }
//      }
//      Transputer.switchStep(transputers, stdout)
//      stdout.flush()
//      stderr.flush()
//    }
//    for (transputer <- transputers) {
//      transputer.printRegisters(stdout)
//      transputer.printRecentMemory(stdout)
//    }
//    if (config.testChecker != null) {
//      testCheckerFileWriter.close()
//      stdout.printf("# Closed log file for testing\n")
//    }
//    if (config.schedChecker != null) {
//      schedCheckerFileWriter.close()
//      stdout.printf("# Closed log file for scheduler checking\n")
//    }
//    if (config.timerChecker != null) {
//      timerCheckerFileWriter.close()
//      stdout.printf("# Closed log file for timer checking\n")
//    }
//    if (config.printWorkspaceMemory) {
//      stdout.printf("# Workspace memory usage\n")
//      for (transputer <- transputers) {
//        transputer.printWorkspaceMemory(stdout)
//      }
//    }
//    stdout.println()
//    stdout.printf("# Total steps: %d\n", loopCount)
//    stdout.printf("\n==DONE==\n")
//    stdout.flush()
//    stderr.flush()
//  }
//}
