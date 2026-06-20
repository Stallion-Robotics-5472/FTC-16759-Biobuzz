package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.List;
import java.util.Map;

/**
 * LocalizationSystem
 *
 * Field-relative robot localization for FTC, fusing:
 *   - GoBilda Pinpoint odometry  (continuous dead-reckoning)
 *   - Limelight 3A AprilTag vision (absolute position corrections)
 *
 * ═══════════════════════════════════════════════════════════
 *  COORDINATE SYSTEM
 * ═══════════════════════════════════════════════════════════
 *   Origin  : center of the FTC field
 *   +X      : "East"  — toward the right wall when facing the Red Alliance
 *   +Y      : "North" — toward the audience wall (away from drivers)
 *   Heading : 0° = facing North (+Y), 90° = East (+X), CCW positive
 *   Units   : inches for X/Y, degrees for heading
 *
 *   An FTC field is 141.7 × 141.7 inches (~12 ft × 12 ft).
 *   Corner coordinates:  (±70.85, ±70.85)
 *
 * ═══════════════════════════════════════════════════════════
 *  QUICK-START
 * ═══════════════════════════════════════════════════════════
 *
 *   // 1. Build the AprilTag field map.
 *   //    Key   = tag ID
 *   //    Value = { fieldX_in, fieldY_in, tagFacingAngle_deg }
 *   //    tagFacingAngle: direction the tag FACE points
 *   //      (0 = North, 90 = East, 180 = South, 270 = West)
 *   Map<Integer, LocalizationSystem.TagPose> tags = new HashMap<>();
 *   tags.put(1, new LocalizationSystem.TagPose( 72.0,  0.0,  90.0));
 *   tags.put(2, new LocalizationSystem.TagPose(-72.0,  0.0, 270.0));
 *
 *   // 2. Construct.
 *   LocalizationSystem loc = new LocalizationSystem(
 *       hardwareMap,
 *       "pinpoint",   3.25, 0.0,       // name; pod X/Y offset from robot center (in)
 *       "limelight",  6.0,  0.0, 0.0,  // name; cam fwd/right offset (in), yaw (deg)
 *       tags,
 *       1.5,                            // min tag area % (0–100) to accept a fix
 *       78.0                            // max trusted tag distance (inches)
 *   );
 *
 *   // 3. Seed starting pose ONCE before your loop.
 *   loc.setInitialPose(-60.0, -60.0, 0.0);
 *
 *   // 4. In the loop — call update() first, then read.
 *   loc.update();
 *   double x   = loc.getX();
 *   double y   = loc.getY();
 *   double hdg = loc.getHeading();
 */
public final class LocalizationSubsystem {

    // ═══════════════════════════════════════════════════════
    //  Public data types
    // ═══════════════════════════════════════════════════════

    /**
     * Known field position of one AprilTag.
     *
     * x          Tag center X from field center, inches, positive = East.
     * y          Tag center Y from field center, inches, positive = North.
     * headingDeg Direction the tag FACE points, degrees CCW from North.
     *                   A robot directly in front of this tag looks in this direction
     *                   toward the tag.  (0 = North, 90 = East, 180 = South, 270 = West)
     */
    public static final class TagPose {
        public final double x;
        public final double y;
        public final double headingDeg;

        public TagPose(double x, double y, double headingDeg) {
            this.x          = x;
            this.y          = y;
            this.headingDeg = headingDeg;
        }
    }

    /** Which sensor drove the most recent pose estimate. */
    public enum PoseSource { VISION, ODOMETRY }

    // ═══════════════════════════════════════════════════════
    //  Constants
    // ═══════════════════════════════════════════════════════

    /** mm → inches (exact by definition). */
    private static final double MM_TO_IN = 1.0 / 25.4;

    /**
     * Vision complementary-filter weight (α).
     *   1.0 = snap entirely to the vision fix each cycle.
     *   0.0 = ignore vision entirely.
     * 0.85 works well in practice; reduce if you see positional jitter.
     */
    private static final double VISION_ALPHA = 0.85;
    private static final double VISION_BETA  = 1.0 - VISION_ALPHA; // pre-computed complement

    // ═══════════════════════════════════════════════════════
    //  Hardware
    // ═══════════════════════════════════════════════════════

    private final GoBildaPinpointDriver pinpoint;
    private final Limelight3A           limelight;

    // ═══════════════════════════════════════════════════════
    //  Configuration — all fixed at construction, never mutated
    // ═══════════════════════════════════════════════════════

    /**
     * Camera mount offsets from robot center, in inches.
     * camForward_in: positive = camera is ahead of center along robot's forward axis.
     * camRight_in  : positive = camera is to the right of center.
     *
     * Both are pre-rotated into robot-frame unit vectors so the per-tag loop
     * only needs to apply the robot heading rotation, not a compound rotation.
     */
    private final double camForward_in;
    private final double camRight_in;

    /**
     * Camera yaw relative to robot heading (degrees, CCW positive).
     * Pre-converted to radians and stored; the raw degree value is kept only
     * for the heading estimation formula where degree arithmetic is needed.
     */
    private final double camYawDeg;
    private final double camYawRad; // pre-computed: Math.toRadians(camYawDeg)

    /** AprilTag field map. Stored by reference — no defensive copy. */
    private final Map<Integer, TagPose> aprilTagMap;

    /**
     * Squared max-distance threshold in inches².
     * Comparing distSq to this avoids a sqrt() in the per-tag filter.
     */
    private final double maxTagDistSq_in;

    private final double visionConfidenceThreshold; // minimum getTargetArea() % to accept a tag fix

    // ═══════════════════════════════════════════════════════
    //  Runtime state — primitives only; zero heap per loop cycle
    // ═══════════════════════════════════════════════════════

    private double x_in    = 0; // field X, inches, East positive
    private double y_in    = 0; // field Y, inches, North positive
    private double hdg_deg = 0; // heading, degrees CCW from North; 0 = facing North

    /**
     * Pinpoint accumulated-pose snapshot from the previous update cycle.
     *
     * IMPORTANT: prevOdoHdg tracks the Pinpoint's OWN accumulated heading
     * (which resets to 0 after resetPosAndIMU()), NOT the field heading.
     * The field heading offset is handled entirely by hdg_deg.
     */
    private double prevOdoX_mm = 0;
    private double prevOdoY_mm = 0;
    private double prevOdoHdg  = 0; // Pinpoint frame, not field frame

    private boolean   initialized = false;
    private PoseSource lastSource = PoseSource.ODOMETRY;

    // ═══════════════════════════════════════════════════════
    //  Constructor
    // ═══════════════════════════════════════════════════════

    /**
     * Constructs and fully initialises the LocalizationSystem.
     * Call {@link #setInitialPose} before the first {@link #update}.
     *
     * @param hardwareMap               FTC HardwareMap from your OpMode.
     * @param pinpointName              HardwareMap name of the GoBilda Pinpoint.
     * @param pinpointXOffset_in        Pinpoint X pod offset from robot center (inches, +right).
     * @param pinpointYOffset_in        Pinpoint Y pod offset from robot center (inches, +forward).
     * @param limelightName             HardwareMap name of the Limelight 3A.
     * @param cameraForwardOffset_in    Camera lens distance ahead of robot center (inches, +forward).
     * @param cameraRightOffset_in      Camera lens distance right of robot center (inches, +right).
     * @param cameraYawOffset_deg       Camera yaw relative to robot heading, CCW positive (degrees).
     * @param aprilTagMap               Map: tag ID → {@link TagPose} for every tag on the field.
     * @param visionConfidenceThreshold Min tag area % (0–100) a detection must occupy in the
     *                                  image to be trusted. Larger = closer = more accurate.
     *                                  Typical value: 1.0–3.0. Tags below this threshold are
     *                                  ignored regardless of distance.
     * @param maxTagDistance_in         Max tag distance (inches) at which vision fixes are trusted.
     */
    public LocalizationSubsystem(
            HardwareMap hardwareMap,
            String pinpointName,
            double pinpointXOffset_in,
            double pinpointYOffset_in,
            String limelightName,
            double cameraForwardOffset_in,
            double cameraRightOffset_in,
            double cameraYawOffset_deg,
            Map<Integer, TagPose> aprilTagMap,
            double visionConfidenceThreshold,
            double maxTagDistance_in
    ) {
        // ── Hardware ─────────────────────────────────────────────────────────
        pinpoint  = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        limelight = hardwareMap.get(Limelight3A.class, limelightName);

        // ── Pinpoint: pod offsets must be supplied in mm to the driver API ────
        pinpoint.setOffsets(
                pinpointXOffset_in * 25.4,
                pinpointYOffset_in * 25.4,
                DistanceUnit.INCH
        );
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();

        // ── Camera mount ──────────────────────────────────────────────────────
        this.camForward_in = cameraForwardOffset_in;
        this.camRight_in   = cameraRightOffset_in;
        this.camYawDeg     = cameraYawOffset_deg;
        this.camYawRad     = Math.toRadians(cameraYawOffset_deg); // pre-computed once

        // ── Vision parameters ─────────────────────────────────────────────────
        this.aprilTagMap               = aprilTagMap;
        this.visionConfidenceThreshold = visionConfidenceThreshold;
        this.maxTagDistSq_in           = maxTagDistance_in * maxTagDistance_in;

        // ── Limelight: start on AprilTag pipeline (index 0 by default) ────────
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // ═══════════════════════════════════════════════════════
    //  Public API
    // ═══════════════════════════════════════════════════════

    /**
     * Seeds the localization system with a known starting pose.
     * <b>Must be called before {@link #update()}.</b>
     * Typically called in {@code init()} or just before the match starts.
     *
     * @param x_in       Field X in inches (positive = East).
     * @param y_in       Field Y in inches (positive = North).
     * @param headingDeg Robot heading in degrees CCW from North (0° = facing North).
     */
    public void setInitialPose(double x_in, double y_in, double headingDeg) {
        this.x_in    = x_in;
        this.y_in    = y_in;
        this.hdg_deg = headingDeg;

        // Reset Pinpoint integrator. After this call the Pinpoint's internal
        // X, Y, and heading all return to 0. We track field heading separately
        // in hdg_deg, so prevOdoHdg must also reset to 0 to match.
        pinpoint.resetPosAndIMU();
        prevOdoX_mm = 0;
        prevOdoY_mm = 0;
        prevOdoHdg  = 0; // Pinpoint frame resets to 0 — NOT headingDeg

        initialized = true;
        lastSource  = PoseSource.ODOMETRY;
    }

    /**
     * Updates the pose estimate. <b>Call exactly once per op-mode loop.</b>
     *
     * Step 1 — integrates the Pinpoint odometry delta into the field pose.
     * Step 2 — if the Limelight has a fresh, trustworthy AprilTag fix, blends
     *           it into the pose via a complementary filter.
     */
    public void update() {
        if (!initialized) return;

        // ── Step 1: Odometry delta integration ──────────────────────────────
        //
        // The Pinpoint accumulates its own X/Y/heading from startup (or last
        // reset).  We compute the delta since the previous cycle and integrate
        // it into the field-frame pose.  Tracking deltas (rather than resetting
        // the Pinpoint each cycle) avoids I2C write overhead and floating-point
        // drift from repeated resets.
        //
        // Pinpoint pose convention (GoBilda default):
        //   X   = robot forward axis (accumulated displacement)
        //   Y   = robot LEFT axis (accumulated displacement)
        //   hdg = robot heading, CCW positive, resets to 0 after resetPosAndIMU()
        //
        // Field convention:
        //   +X = East,   +Y = North,  heading 0 = facing North
        //
        // Mapping robot axes to robot-frame vectors:
        //   robot_fwd   = +dX_in   (Pinpoint +X is forward)
        //   robot_right = -dY_in   (Pinpoint +Y is LEFT, so right = -Y)

        pinpoint.update();
        final Pose2D odo      = pinpoint.getPosition();
        final double curX_mm  = odo.getX(DistanceUnit.MM);
        final double curY_mm  = odo.getY(DistanceUnit.MM);
        final double curHdg   = odo.getHeading(AngleUnit.DEGREES); // Pinpoint frame

        final double dX_in = (curX_mm - prevOdoX_mm) * MM_TO_IN;
        final double dY_in = (curY_mm - prevOdoY_mm) * MM_TO_IN;
        final double dHdg  = normalizeAngle(curHdg - prevOdoHdg);

        prevOdoX_mm = curX_mm;
        prevOdoY_mm = curY_mm;
        prevOdoHdg  = curHdg;

        // Integrate heading first; use the midpoint-arc heading for the
        // displacement rotation (reduces error during sharp turns).
        final double midHdgRad = Math.toRadians(hdg_deg + dHdg * 0.5);
        final double cosMid    = Math.cos(midHdgRad);
        final double sinMid    = Math.sin(midHdgRad);

        hdg_deg = normalizeAngle(hdg_deg + dHdg);

        // robot_fwd = dX_in, robot_right = -dY_in
        // Rotation matrix for North-up, East-right, heading CCW from North:
        //   field_East  (dX_field) =  robot_right * cosMid + robot_fwd * sinMid
        //   field_North (dY_field) = -robot_right * sinMid + robot_fwd * cosMid
        final double fwd   =  dX_in;
        final double right = -dY_in;
        x_in += right * cosMid + fwd * sinMid;
        y_in += -right * sinMid + fwd * cosMid;

        lastSource = PoseSource.ODOMETRY;

        // ── Step 2: Vision correction ─────────────────────────────────────────
        applyVisionCorrection();
    }

    /** @return Current field X in inches (positive = East). */
    public double getX()       { return x_in; }

    /** @return Current field Y in inches (positive = North). */
    public double getY()       { return y_in; }

    /**
     * @return Current robot heading in degrees, CCW from North.
     *         0° = facing North, 90° = facing East, -90° = facing West.
     */
    public double getHeading() { return hdg_deg; }

    /** @return Which sensor last updated the pose estimate. */
    public PoseSource getLastPoseSource() { return lastSource; }

    /**
     * Hard-resets the pose mid-match (e.g. after a collision or manual re-seed).
     * Identical to {@link #setInitialPose}.
     */
    public void resetPose(double x_in, double y_in, double headingDeg) {
        setInitialPose(x_in, y_in, headingDeg);
    }

    /** Adds or replaces a tag in the field map at runtime. */
    public void putAprilTag(int id, TagPose pose) {
        aprilTagMap.put(id, pose);
    }

    /** Stops the Limelight. Call from your OpMode's {@code stop()}. */
    public void stop() {
        limelight.stop();
    }

    // ═══════════════════════════════════════════════════════
    //  Vision — private
    // ═══════════════════════════════════════════════════════

    /**
     * Queries the Limelight for AprilTag detections.  Every tag that passes
     * area and distance filters contributes an independent full-pose estimate;
     * all passing estimates are averaged, then blended into the current pose
     * with a complementary filter.
     *
     * API notes:
     *   getRobotPoseTargetSpace() → Pose3D
     *     .getPosition()           → Position  (fields .x .y .z in metres by default;
     *                                            use .toUnit(DistanceUnit.INCH) to convert)
     *     .getOrientation()        → YawPitchRollAngles
     *                                  .getYaw(AngleUnit.DEGREES) = camera yaw re: tag normal
     *
     *   Limelight target-space axes (getRobotPoseTargetSpace):
     *     +X = tag's right   (camera's left  when facing tag)
     *     +Y = tag's up
     *     +Z = tag's forward (toward the camera)
     *   So the camera-to-tag vector in camera frame is (−X, Z) for (right, forward).
     *
     * Quality filter: getTargetArea() returns the tag's percentage of image area (0–100).
     *   Larger area = closer tag = more reliable pose estimate.
     *   No getConfidence() method exists on FiducialResult.
     *
     * Performance profile (no tags visible — most common case):
     *   getLatestResult() — one non-blocking call; immediate return on null/invalid.
     *
     * Performance profile (N tags visible):
     *   Trig per tag: one atan2, one sqrt (reused from distSq), two sin/cos pairs.
     *   camYawRad and robotHdgRad/cosR/sinR are pre-computed outside the loop.
     *   No heap allocation in steady state (Pose3D/Position objects are SDK-owned).
     */
    private void applyVisionCorrection() {
        final LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        final List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        // Pre-compute robot heading trig once for all tags this cycle.
        final double robotHdgRad = Math.toRadians(hdg_deg);
        final double cosR        = Math.cos(robotHdgRad);
        final double sinR        = Math.sin(robotHdgRad);

        // Accumulators — heading via circular mean to handle ±180° wrap correctly.
        double sumX      = 0;
        double sumY      = 0;
        double sumSinHdg = 0;
        double sumCosHdg = 0;
        int    count     = 0;

        final int n = tags.size();
        for (int i = 0; i < n; i++) {
            final LLResultTypes.FiducialResult tag = tags.get(i);

            // ── Field pose lookup ─────────────────────────────────────────────
            final TagPose tagField = aprilTagMap.get(tag.getFiducialId());
            if (tagField == null) continue;

            // ── Area filter — proxy for detection quality and distance ─────────
            // getTargetArea() is 0–100 (% of image). No getConfidence() exists.
            // Larger area → tag is closer and detection is more reliable.
            if (tag.getTargetArea() < visionConfidenceThreshold) continue;

            // ── Camera-to-tag 3D pose from Limelight ──────────────────────────
            // getRobotPoseTargetSpace() returns the ROBOT's pose in the TAG's
            // coordinate frame.  We want the inverse: tag's position relative to
            // the camera.  However, for our 2-D localization purposes what we
            // actually need is just the horizontal displacement (X and Z in tag
            // space), which equals the camera-to-tag vector projected onto the
            // ground plane.
            //
            // Tag-space axes (Limelight convention):
            //   +X = tag's right,  +Y = tag's up,  +Z = toward camera (out of tag face)
            //
            // So:  tag_right_of_cam = -pose.x   (tag's right is camera's left)
            //       tag_fwd_of_cam  =  pose.z   (tag Z points toward camera)
            //
            // Position is returned in metres; convert to inches via toUnit().
            final Pose3D targetPose = tag.getRobotPoseTargetSpace();
            if (targetPose == null) continue;

            final Position pos = targetPose.getPosition().toUnit(DistanceUnit.INCH);
            // In tag space: pos.x = tag's rightward component, pos.z = depth toward camera.
            // Camera-frame vector to tag: right = -pos.x, forward = pos.z
            final double tx_in = -pos.x; // camera-right component to tag
            final double tz_in =  pos.z; // camera-forward component to tag

            // ── Distance filter — squared comparison avoids sqrt ───────────────
            final double distSq_in = tx_in * tx_in + tz_in * tz_in;
            if (distSq_in > maxTagDistSq_in) continue;

            // ═════════════════════════════════════════════════════════════════
            //  Robot center field position from this tag
            // ═════════════════════════════════════════════════════════════════
            //
            // bearingInCam: CCW angle of tag from camera's forward axis.
            //   atan2(-tx, tz):  tx right is CW, so negate for CCW convention.
            //
            // tagBearingRad: bearing of tag in field frame.
            //   = robot heading + camera yaw mount offset + angle within camera frame
            //   (camYawRad pre-computed at construction)
            //
            // Camera field position:
            //   camX = tagField.x − dist * sin(tagBearing)   [East component]
            //   camY = tagField.y − dist * cos(tagBearing)   [North component]
            //   (sin for East, cos for North because 0° = North convention)
            //
            // Robot center = camera − mount offset rotated into field frame:
            //   offset_East  = camForward_in * sinR + camRight_in * cosR
            //   offset_North = camForward_in * cosR − camRight_in * sinR
            //   (cosR, sinR pre-computed once before this loop)

            final double bearingInCam  = Math.atan2(-tx_in, tz_in);
            final double tagBearingRad = robotHdgRad + camYawRad + bearingInCam;
            final double dist_in       = Math.sqrt(distSq_in);
            final double sinBear       = Math.sin(tagBearingRad);
            final double cosBear       = Math.cos(tagBearingRad);

            final double camX   = tagField.x - dist_in * sinBear;
            final double camY   = tagField.y - dist_in * cosBear;

            final double robotX = camX - (camForward_in * sinR + camRight_in * cosR);
            final double robotY = camY - (camForward_in * cosR - camRight_in * sinR);

            // ── Heading from tag ──────────────────────────────────────────────
            // getOrientation().getYaw() gives the camera's yaw relative to the
            // tag's surface normal (positive CCW).
            // robotHdg = tagFacingDir − cameraYawReTag − cameraMountYaw
            final YawPitchRollAngles orient = targetPose.getOrientation();
            final double ryDeg  = (orient != null) ? orient.getYaw(AngleUnit.DEGREES) : 0.0;
            final double estHdg = normalizeAngle(tagField.headingDeg - ryDeg - camYawDeg);

            // Accumulate via circular mean to handle ±180° wrap correctly.
            sumX      += robotX;
            sumY      += robotY;
            sumSinHdg += Math.sin(Math.toRadians(estHdg));
            sumCosHdg += Math.cos(Math.toRadians(estHdg));
            count++;
        }

        if (count == 0) return;

        // ── Average all passing tag estimates ─────────────────────────────────
        final double invCount = 1.0 / count;
        final double visX   = sumX * invCount;
        final double visY   = sumY * invCount;
        final double visHdg = Math.toDegrees(Math.atan2(sumSinHdg, sumCosHdg));

        // ── Complementary filter blend ────────────────────────────────────────
        // X/Y: standard weighted blend.
        // Heading: circular-mean blend to avoid ±180° discontinuity in the mix.
        x_in = VISION_ALPHA * visX + VISION_BETA * x_in;
        y_in = VISION_ALPHA * visY + VISION_BETA * y_in;

        final double sinV = Math.sin(Math.toRadians(visHdg));
        final double cosV = Math.cos(Math.toRadians(visHdg));
        final double sinO = Math.sin(Math.toRadians(hdg_deg));
        final double cosO = Math.cos(Math.toRadians(hdg_deg));
        hdg_deg = Math.toDegrees(Math.atan2(
                VISION_ALPHA * sinV + VISION_BETA * sinO,
                VISION_ALPHA * cosV + VISION_BETA * cosO));

        lastSource = PoseSource.VISION;
    }

    // ═══════════════════════════════════════════════════════
    //  Math helpers
    // ═══════════════════════════════════════════════════════

    /**
     * Normalizes {@code deg} to the half-open interval (−180°, 180°].
     * A single modulo plus at most one branch — faster than a loop.
     */
    private static double normalizeAngle(double deg) {
        deg %= 360.0;
        if (deg >  180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }
}