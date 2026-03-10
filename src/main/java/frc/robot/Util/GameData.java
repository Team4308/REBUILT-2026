package frc.robot.Util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameData {
    public static boolean isHubActive(boolean opponent) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };
        
        // If we are tracking the opposing hub, it would be opposite of ours
        if (opponent) {
            shift1Active = !shift1Active;
        }

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    public static double timeToNextPhase() {
        double matchTime = DriverStation.getMatchTime();
        if (matchTime > 130) {
            return matchTime - 130;
        } else if (matchTime > 105) {
            return matchTime - 105;
        } else if (matchTime > 80) {
            return matchTime - 80;
        } else if (matchTime > 55) {
            return matchTime - 55;
        } else if (matchTime > 30) {
            return matchTime - 30;
        } else {
            return matchTime;
        }
    }
}
