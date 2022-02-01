// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;

/**
 * This class allows us to interrupt a command group and then start from the same point again
 * https://github.com/Rob-Heslin/SenableCommandGroupTest/blob/main/src/main/java/edu/wpi/first/wpilibj2/command/SendableCommandGroup3.java
 * TODO: This is a work in progress
 */
public class InterruptibleCommandGroup extends CommandGroupBase {
  private final List<Command> m_commands = new ArrayList<>();
  private final List<String> m_commandNames = new ArrayList<>();
  private static int m_currentCommandIndex = -1;
  private boolean m_runWhenDisabled = true;

  /** Creates a new InterruptibleCommandGroup. */
  public InterruptibleCommandGroup(Command... commands) {
    addCommands(commands);
  }

  @Override
  public final void addCommands(Command... commands) {
    requireUngrouped(commands);

    if (m_currentCommandIndex != -1) {
      throw new IllegalStateException(
          "Commands cannot be added to a CommandGroup while the group is running");
    }

    // TODO: this method is not visible to the child class
    // registerGroupedCommands(commands);
    
    int commandCount = 0;
    for (Command command : commands) {
      m_commands.add(command);
      m_commandNames.add(commandCount + command.getName());
      commandCount++;
      m_requirements.addAll(command.getRequirements());
      m_runWhenDisabled &= command.runsWhenDisabled();
    }
    postCurrentCommand();
  }
  
  @Override
  public void initialize() {
    if(m_currentCommandIndex == -1) {
      m_currentCommandIndex = 0;
    }

    if (!m_commands.isEmpty()) {
      m_commands.get(m_currentCommandIndex).initialize();
    }
    postCurrentCommand();
  }

  @Override
  public void execute() {
    if (m_commands.isEmpty()) {
      return;
    }

    Command currentCommand = m_commands.get(m_currentCommandIndex);

    currentCommand.execute();
    if (currentCommand.isFinished()) {
      currentCommand.end(false);
      m_currentCommandIndex++;
      if (m_currentCommandIndex < m_commands.size()) {
        m_commands.get(m_currentCommandIndex).initialize();
        postCurrentCommand();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted
        && !m_commands.isEmpty()
        && m_currentCommandIndex > -1
        && m_currentCommandIndex < m_commands.size()) {
      m_commands.get(m_currentCommandIndex).end(true);
    }else{
      m_currentCommandIndex = -1;
    }
    postCurrentCommand();
  }

  @Override
  public boolean isFinished() {
    return m_currentCommandIndex == m_commands.size();
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }

  public final void onePrevious(){
    m_currentCommandIndex--;
    postCurrentCommand();
  }

  public final void oneLater(){
    m_currentCommandIndex++;
    postCurrentCommand();
  }

  private final void postCurrentCommand(){
    String output = "";
    if(m_currentCommandIndex == -1){
      output = "NOT_STARTED";
    }else{
      output = m_commandNames.get(m_currentCommandIndex);
    }
    SmartDashboard.putString(this.getName() + " Running:",output);
  }
}
