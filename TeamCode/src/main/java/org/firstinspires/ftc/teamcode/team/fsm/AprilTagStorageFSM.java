package org.firstinspires.ftc.teamcode.team.fsm;

/**
 * Static storage for persisting the scanned motif AprilTag between Auto and TeleOp.
 *
 * Notes:
 * - This persists only while the Robot Controller app process stays alive.
 * - It will reset if the RC app is restarted.
 */
public class AprilTagStorageFSM {

    /**
     * The motif AprilTag id that was scanned in Auto.
     * Expected values are 21, 22, or 23.
     *
     * -1 means "no motif stored".
     */
    public static int motifTagId = -1;

    /** True once Auto has successfully saved a motif id. */
    public static boolean autoScanComplete = false;

    /** Reset method to clear data (call at start of auto). */
    public static void reset() {
        motifTagId = -1;
        autoScanComplete = false;
    }

    /**
     * Save a scanned AprilTag id as the motif.
     * Only accepts motif tags (21/22/23).
     */
    public static void saveMotifTagId(int tagId) {
        if (isMotifTag(tagId)) {
            motifTagId = tagId;
            autoScanComplete = true;
        } else {
            // Defensive: don't mark complete if the tag isn't a motif tag.
            reset();
        }
    }

    /** True if tagId is one of the motif AprilTags (21, 22, 23). */
    public static boolean isMotifTag(int tagId) {
        return tagId == 21 || tagId == 22 || tagId == 23;
    }

    public static boolean hasValidScan() {
        return autoScanComplete && isMotifTag(motifTagId);
    }
}