// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/** Add your docs here. */
public class CommandUtils {
    public Command CustomChainCommand(Boolean firstCondition, Command firstCommand, Command secondCommand) {
        return new WaitUntilCommand(() -> firstCondition)
        .deadlineWith(firstCommand)
        .andThen(secondCommand); 
    }
}
