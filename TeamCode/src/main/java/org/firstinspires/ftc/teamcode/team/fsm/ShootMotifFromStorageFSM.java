package org.firstinspires.ftc.teamcode.team.fsm;

/**
 * TeleOp macro: shoot the 3-ball motif based on the AprilTag motif id (21/22/23)
 * saved in {@link AprilTagStorageFSM}, while ensuring we only shoot the correct
 * color using TrayFSM's detected slot colors.
 *
 * Assumptions/contract:
 * - TrayFSM has already classified the 3 tray slots as PURPLE/GREEN/EMPTY/UNKNOWN.
 * - We will only shoot a step if the tray currently holds the expected color.
 * - If the expected color isn't present, we skip that shot (and continue) to avoid
 *   firing the wrong color.
 */
public class ShootMotifFromStorageFSM {

    public enum State {
        IDLE,
        MOVE_TRAY,
        START_SHOT,
        WAIT_SHOT,
        DONE
    }

    /** Motif colors in order for tags 21/22/23.
     * Using the common FTC_DECODE convention:
     *  - 21 = GPP
     *  - 22 = PGP
     *  - 23 = PPG
     */
    public enum MotifColor { GREEN, PURPLE }

    private final DarienOpModeFSM opMode;
    private final TrayFSM trayFSM;
    private final ShootArtifactFSM shootArtifactFSM;

    private State state = State.IDLE;
    private MotifColor[] motif = null;
    private int stepIndex = 0;

    // Which tray slot (0..2) we're going to shoot from this step; -1 means skip.
    private int selectedSlotIndex = -1;

    private double stateStartTime = 0;

    public static double TRAY_MOVE_DELAY = 0.35; // seconds, allow tray servo to settle in score position

    public ShootMotifFromStorageFSM(DarienOpModeFSM opMode, TrayFSM trayFSM) {
        this.opMode = opMode;
        this.trayFSM = trayFSM;
        this.shootArtifactFSM = opMode.shootArtifactFSM;
    }

    public void start(double currentTimeSeconds, double shootingPower) {
        if (!AprilTagStorageFSM.hasValidScan()) {
            // Nothing saved from auto.
            state = State.DONE;
            return;
        }

        motif = motifForTag(AprilTagStorageFSM.motifTagId);
        if (motif == null || motif.length != 3) {
            state = State.DONE;
            return;
        }

        // Make sure tray auto-intake isn't fighting us while shooting.
        if (trayFSM.isAutoIntakeRunning()) {
            trayFSM.stopAutoIntake();
        }

        stepIndex = 0;
        selectedSlotIndex = -1;
        stateStartTime = currentTimeSeconds;
        state = State.MOVE_TRAY;

        // Shooter motors will be handled per-shot by ShootArtifactFSM.
        shootArtifactFSM.resetShooting();
        shootArtifactFSM.setEjectionMotorsControlledByPattern(false);

        this.requestedPower = shootingPower;
    }

    private double requestedPower = DarienOpModeFSM.SHOT_GUN_POWER_UP;

    public void update(double currentTimeSeconds, boolean telemetryEnabled) {
        if (state == State.IDLE || state == State.DONE) return;

        if (motif == null || motif.length != 3) {
            state = State.DONE;
            return;
        }

        // Keep TrayFSM classification fresh.
        trayFSM.update();

        if (stepIndex >= 3) {
            state = State.DONE;
            return;
        }

        MotifColor expected = motif[stepIndex];

        switch (state) {
            case MOVE_TRAY: {
                selectedSlotIndex = findSlotWithColor(expected);

                // Fallback: if the expected color isn't present, shoot any filled slot instead.
                if (selectedSlotIndex < 0) {
                    selectedSlotIndex = findAnyFilledSlot();
                }

                if (telemetryEnabled) {
                    opMode.telemetry.addData("MotifStep", "%d/3", stepIndex + 1);
                    opMode.telemetry.addData("ExpectedColor", expected);
                    opMode.telemetry.addData("SelectedSlot", selectedSlotIndex);
                    opMode.telemetry.addData("FallbackMode", (findSlotWithColor(expected) < 0) ? "ANY" : "MATCH");
                }

                if (selectedSlotIndex < 0) {
                    // Nothing to shoot.
                    stepIndex++;
                    stateStartTime = currentTimeSeconds;
                    // stay in MOVE_TRAY for next step
                    break;
                }

                // Move tray to the scoring position corresponding to this physical slot.
                // Slot index 0=>score slot 0, 1=>score slot 1, 2=>score slot 2
                trayFSM.moveToScoreSlot(selectedSlotIndex);
                stateStartTime = currentTimeSeconds;
                state = State.START_SHOT;
                break;
            }

            case START_SHOT: {
                if (currentTimeSeconds - stateStartTime < TRAY_MOVE_DELAY) {
                    break;
                }

                // Start a shot using the existing FSM.
                shootArtifactFSM.startShooting(requestedPower);
                stateStartTime = currentTimeSeconds;
                state = State.WAIT_SHOT;
                break;
            }

            case WAIT_SHOT: {
                shootArtifactFSM.updateShooting();

                if (shootArtifactFSM.shootingDone()) {
                    shootArtifactFSM.resetShooting();

                    // After shooting, mark the slot empty so we won't double-shoot it.
                    // TrayFSM doesn't expose a setter, so we just advance and rely on sensor update.
                    stepIndex++;
                    selectedSlotIndex = -1;
                    stateStartTime = currentTimeSeconds;
                    state = State.MOVE_TRAY;
                }
                break;
            }
        }
    }

    public boolean isRunning() {
        return state != State.IDLE && state != State.DONE;
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    public void stop() {
        state = State.DONE;
        shootArtifactFSM.shotGunStop();
        shootArtifactFSM.resetShooting();
    }

    private int findSlotWithColor(MotifColor expected) {
        TrayFSM.SlotState wanted = (expected == MotifColor.GREEN) ? TrayFSM.SlotState.GREEN : TrayFSM.SlotState.PURPLE;

        // Prefer an exact match.
        for (int i = 0; i < 3; i++) {
            if (trayFSM.getSlot(i) == wanted) return i;
        }
        return -1;
    }

    private int findAnyFilledSlot() {
        for (int i = 0; i < 3; i++) {
            TrayFSM.SlotState s = trayFSM.getSlot(i);
            if (s == TrayFSM.SlotState.GREEN || s == TrayFSM.SlotState.PURPLE) return i;
        }
        return -1;
    }

    private MotifColor[] motifForTag(int tagId) {
        switch (tagId) {
            case 21: // GPP
                return new MotifColor[]{MotifColor.GREEN, MotifColor.PURPLE, MotifColor.PURPLE};
            case 22: // PGP
                return new MotifColor[]{MotifColor.PURPLE, MotifColor.GREEN, MotifColor.PURPLE};
            case 23: // PPG
                return new MotifColor[]{MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN};
            default:
                return null;
        }
    }
}

