// mediapipe_pose_controller.js

import { PoseLandmarker, FilesetResolver } from "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision/vision_bundle.js";

export class MediaPipeHandController { // Consider renaming this to MediaPipePoseController for clarity
    constructor(viewerInstance, videoElement) {
        this.viewer = viewerInstance;
        this.video = videoElement;
        this.poseLandmarker = null;
        this.runningMode = "VIDEO";
        this.webcamInitialized = false;
        this.lastVideoTime = -1;

        // Store previous joint values for smoothing
        this.previousJointValues = {};
        this.debugMode = true; // Keep debugMode enabled for detailed logs during testing

        // Define the robot's initial (default) pose for all controlled joints
        this.initialRobotPose = {
            'l_shoulder_x': 0,
            'l_shoulder_y': 0,
            'l_arm_z': 0,
            'l_elbow_y': 0.02, // Straight position for left elbow
            'r_shoulder_x': 0,
            'r_shoulder_y': 0,
            'r_arm_z': 0,
            'r_elbow_y': -0.02, // Straight position for right elbow
            'head_z': 0,
            'head_y': 0
        };

        // History for arm length for foreshortening detection
        this.armLengthHistory_L = [];
        this.armLengthHistory_R = [];

        this.initMediaPipe();
    }

    async initMediaPipe() {
        const vision = await FilesetResolver.forVisionTasks(
            "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@latest/wasm"
        );

        this.poseLandmarker = await PoseLandmarker.createFromOptions(vision, {
            baseOptions: {
                modelAssetPath: "/mediapipe_models/pose_landmarker_heavy.task",
                delegate: "GPU"
            },
            runningMode: this.runningMode,
            numPoses: 1 // Only need to detect one person
        });

        console.log("MediaPipe PoseLandmarker initialized.");
        this.setupWebcam();
    }

    setupWebcam() {
        if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
            navigator.mediaDevices.getUserMedia({ video: true })
                .then((stream) => {
                    this.video.srcObject = stream;
                    this.video.addEventListener("canplay", () => {
                        this.video.play();

                        this.overlayCanvas = document.getElementById("overlay");
                        this.overlayCanvas.width = this.video.videoWidth;
                        this.overlayCanvas.height = this.video.videoHeight;
                        this.overlayCtx = this.overlayCanvas.getContext("2d");

                        this.webcamInitialized = true;
                        console.log("Webcam stream started. Starting pose detection loop.");
                        this.detectPosesInRealTime();
                    }, { once: true });
                })
                .catch((error) => {
                    console.error("Error accessing webcam:", error);
                    this.viewer.updateStatus("Error accessing webcam: " + error.message, 'error');
                });
        } else {
            this.viewer.updateStatus("Webcam not supported by this browser.", 'error');
        }
    }

    async detectPosesInRealTime() {
        if (this.webcamInitialized && this.poseLandmarker) {
            const startTimeMs = performance.now();

            try {
                // Only detect if video has new frame
                if (this.video.currentTime !== this.lastVideoTime) {
                    this.lastVideoTime = this.video.currentTime;
                    const result = await this.poseLandmarker.detectForVideo(this.video, startTimeMs);

                    if (result.landmarks && result.landmarks.length > 0) {
                        const poseLandmarks = result.landmarks[0];
                        
                        // NEW: Check if the full human body is visible
                        if (this.isFullBodyVisible(poseLandmarks)) {
                            this.mapLandmarksToRobot(poseLandmarks);
                            this.drawLandmarks(poseLandmarks); // Draw detected pose
                        } else {
                            // If partial body or not visible enough, reset robot to initial pose
                            if (this.debugMode) console.log('Partial body detected or not visible enough. Resetting to initial pose.');
                            this.resetRobotToInitialPose();
                            this.drawLandmarks(poseLandmarks); // Still draw detected landmarks for feedback
                        }
                    } else {
                        // No landmarks detected at all
                        if (this.debugMode) console.log('No pose landmarks detected. Resetting to initial pose.');
                        this.resetRobotToInitialPose();
                        this.drawLandmarks([]); // Clear overlay if no pose
                    }
                } else {
                    // No new video frame
                    this.drawLandmarks([]); // Clear overlay if video is paused/no new frame
                }
            } catch (error) {
                console.error("Pose detection error:", error);
                // In case of error, revert to initial pose as a safe fallback
                this.resetRobotToInitialPose();
                this.drawLandmarks([]);
            }
        }
        requestAnimationFrame(this.detectPosesInRealTime.bind(this));
    }

    /**
     * Checks if a set of critical body landmarks are sufficiently visible.
     * @param {Array} landmarks - The array of pose landmarks from MediaPipe.
     * @param {number} visibilityThreshold - The minimum visibility score for a landmark to be considered visible (0.0 to 1.0).
     * @returns {boolean} True if all key landmarks are visible, false otherwise.
     */
    isFullBodyVisible(landmarks, visibilityThreshold = 0.6) {
        const POSE = {
            NOSE: 0,
            LEFT_EAR: 7,
            RIGHT_EAR: 8,
            LEFT_SHOULDER: 11,
            RIGHT_SHOULDER: 12,
            LEFT_WRIST: 15,
            RIGHT_WRIST: 16,
            LEFT_HIP: 23,
            RIGHT_HIP: 24,
            LEFT_KNEE: 25,
            RIGHT_KNEE: 26,
            LEFT_ANKLE: 27,
            RIGHT_ANKLE: 28,
            LEFT_FOOT_INDEX: 31, // End of the foot
            RIGHT_FOOT_INDEX: 32 // End of the foot
        };

        const keyLandmarkIndices = [
            POSE.NOSE,
            POSE.LEFT_EAR,
            POSE.RIGHT_EAR,
            POSE.LEFT_SHOULDER,
            POSE.RIGHT_SHOULDER,
            POSE.LEFT_WRIST,
            POSE.RIGHT_WRIST,
            POSE.LEFT_HIP,
            POSE.RIGHT_HIP,
            POSE.LEFT_KNEE,
            POSE.RIGHT_KNEE,
            POSE.LEFT_ANKLE,
            POSE.RIGHT_ANKLE,
            POSE.LEFT_FOOT_INDEX,
            POSE.RIGHT_FOOT_INDEX
        ];

        let allVisible = true;
        if (this.debugMode) console.groupCollapsed('Full Body Visibility Check');

        for (const index of keyLandmarkIndices) {
            const landmark = landmarks[index];
            // Check if landmark exists and its visibility is above the threshold
            if (!landmark || landmark.visibility < visibilityThreshold) {
                if (this.debugMode) {
                    const landmarkName = Object.keys(POSE).find(key => POSE[key] === index) || `Landmark ${index}`;
                    console.log(`   ${landmarkName} (idx:${index}) visibility: ${landmark ? landmark.visibility.toFixed(2) : 'N/A'} < ${visibilityThreshold.toFixed(2)}`);
                }
                allVisible = false;
                break; // No need to check further if one is not visible
            }
        }
        if (this.debugMode) {
            console.log(`Overall Full Body Visible: ${allVisible}`);
            console.groupEnd();
        }
        return allVisible;
    }

    /**
     * Resets all controlled robot joints to their predefined initial/default values.
     * Also resets the smoothing history for these joints to ensure a clean start.
     */
    resetRobotToInitialPose() {
        for (const jointName in this.initialRobotPose) {
            this.viewer?.updateJoint(jointName, this.initialRobotPose[jointName]);
            // Also reset the smoothed value for consistency to prevent jumpiness on re-entry
            this.previousJointValues[jointName] = this.initialRobotPose[jointName];
        }
    }

    drawLandmarks(landmarks) {
        this.overlayCtx.clearRect(0, 0, this.overlayCanvas.width, this.overlayCanvas.height);

        if (landmarks.length === 0) { // If no landmarks, just clear and return
            return;
        }

        // Save the current state of the canvas
        this.overlayCtx.save();
        
        // Flip the canvas horizontally to un-mirror the drawing
        this.overlayCtx.translate(this.overlayCanvas.width, 0);
        this.overlayCtx.scale(-1, 1);

        const scaleX = this.overlayCanvas.width;
        const scaleY = this.overlayCanvas.height;

        // Define connections for all relevant body parts
        const connections = [
            // Head
            [0, 1], [0, 4], [1, 2], [2, 3], [4, 5], [5, 6], [9, 10], // Face landmarks
            [0, 7], [0, 8], // Nose to ears (simplified for visual)

            // Torso
            [11, 12], // Shoulders
            [23, 24], // Hips
            [11, 23], [12, 24], // Shoulders to Hips (torso sides)
            [11, 13], [13, 15], // Left Arm
            [12, 14], [14, 16], // Right Arm

            // Legs
            [23, 25], [25, 27], // Left Leg: Hip to Knee to Ankle
            [24, 26], [26, 28], // Right Leg: Hip to Knee to Ankle
            [27, 29], [29, 31], // Left Foot: Ankle to Heel to Foot Index
            [28, 30], [30, 32]  // Right Foot: Ankle to Heel to Foot Index
        ];

        this.overlayCtx.strokeStyle = '#33aaff'; // A vibrant blue for lines
        this.overlayCtx.lineWidth = 5; // Increased line width for connections
        for (const connection of connections) {
            const start = landmarks[connection[0]];
            const end = landmarks[connection[1]];
            
            // Only draw connection if both landmarks are valid and sufficiently visible
            if (start && end && 
                (start.visibility === undefined || start.visibility > 0.5) &&
                (end.visibility === undefined || end.visibility > 0.5)) {
                this.overlayCtx.beginPath();
                this.overlayCtx.moveTo(start.x * scaleX, start.y * scaleY);
                this.overlayCtx.lineTo(end.x * scaleX, end.y * scaleY);
                this.overlayCtx.stroke();
            }
        }

        // Draw individual landmarks (points)
        this.overlayCtx.fillStyle = '#FF4136'; // Red for landmarks
        this.overlayCtx.strokeStyle = '#FFFFFF'; // White border
        this.overlayCtx.lineWidth = 2; // Increased line width for landmark borders
        for (let i = 0; i < landmarks.length; i++) {
            // Only draw if landmark exists and visibility is good
            if (landmarks[i] && (landmarks[i].visibility === undefined || landmarks[i].visibility > 0.5)) { 
                const x = landmarks[i].x * scaleX;
                const y = landmarks[i].y * scaleY;
                
                this.overlayCtx.beginPath();
                this.overlayCtx.arc(x, y, 7, 0, 2 * Math.PI); // Increased radius for landmarks
                this.overlayCtx.fill();
                this.overlayCtx.stroke(); // Draw border
            }
        }

        // Restore the canvas state
        this.overlayCtx.restore();
    }


    mapLandmarksToRobot(poseLandmarks) {
        const POSE = {
            NOSE: 0,
            LEFT_EYE_INNER: 1,
            LEFT_EYE: 2,
            LEFT_EYE_OUTER: 3,
            RIGHT_EYE_INNER: 4,
            RIGHT_EYE: 5,
            RIGHT_EYE_OUTER: 6,
            LEFT_EAR: 7,
            RIGHT_EAR: 8,
            LEFT_SHOULDER: 11,
            LEFT_ELBOW: 13,
            LEFT_WRIST: 15,
            RIGHT_SHOULDER: 12,
            RIGHT_ELBOW: 14,
            RIGHT_WRIST: 16
        };
    
        // Helper functions for 2D operations only
        const create2DVector = (p1, p2) => ({
            x: p2.x - p1.x,
            y: p2.y - p1.y
        });
    
        const angle2D = (vec) => Math.atan2(vec.y, vec.x);
        
        // Function to calculate angle between two vectors (e.g., upper arm and forearm)
        // This calculates the internal angle at the elbow.
        const angleBetweenVectors = (vec1, vec2) => {
            const dotProduct = vec1.x * vec2.x + vec1.y * vec2.y;
            const magnitude1 = vectorMagnitude(vec1);
            const magnitude2 = vectorMagnitude(vec2);
            if (magnitude1 === 0 || magnitude2 === 0) return 0; // Avoid division by zero
            const clampedDotProduct = Math.max(-1, Math.min(1, dotProduct / (magnitude1 * magnitude2)));
            return Math.acos(clampedDotProduct);
        };
    
        const vectorMagnitude = (vec) => Math.hypot(vec.x, vec.y);
    
        // Smoothing
        if (!this.previousJointValues) this.previousJointValues = {};
        const smooth = (jointName, value, alpha = 0.2) => {
            if (isNaN(value)) {
                if (this.debugMode) console.warn(`Smoothing: Input value for ${jointName} is NaN. Using previous or default.`);
                return this.previousJointValues[jointName] || (this.initialRobotPose[jointName] || 0); // Handle NaN by returning previous or initial value
            }
            const prev = this.previousJointValues[jointName] ?? value; // Use value if no previous
            const smoothed = prev * (1 - alpha) + value * alpha;
            this.previousJointValues[jointName] = smoothed;
            return smoothed;
        };
    
        // Landmark validation (visibility check for individual parts - still useful as a fallback for specific limbs)
        const isValid = (landmark) => landmark && 
            (landmark.visibility === undefined || landmark.visibility > 0.5); // Check visibility if available
    
        // --- Left Hand Logic ---
        if (!isValid(poseLandmarks[POSE.LEFT_SHOULDER]) || 
            !isValid(poseLandmarks[POSE.LEFT_ELBOW]) || 
            !isValid(poseLandmarks[POSE.LEFT_WRIST])) {
            if (this.debugMode) console.warn("Invalid left hand landmarks, defaulting to initial pose for left arm.");
            // Use initial pose values as safe defaults if a limb is invalid
            this.viewer?.updateJoint('l_shoulder_x', this.initialRobotPose.l_shoulder_x);
            this.viewer?.updateJoint('l_shoulder_y', this.initialRobotPose.l_shoulder_y);
            this.viewer?.updateJoint('l_arm_z', this.initialRobotPose.l_arm_z);
            this.viewer?.updateJoint('l_elbow_y', this.initialRobotPose.l_elbow_y);
        } else {
            const shoulder = poseLandmarks[POSE.LEFT_SHOULDER];
            const elbow = poseLandmarks[POSE.LEFT_ELBOW];
            const wrist = poseLandmarks[POSE.LEFT_WRIST];

            if (this.debugMode) {
                console.groupCollapsed('Left Arm Debug (Raw Landmarks)');
                console.log(`L_Shoulder (11): x:${shoulder.x.toFixed(4)}, y:${shoulder.y.toFixed(4)}, z:${shoulder.z ? shoulder.z.toFixed(4) : 'N/A'}, vis:${shoulder.visibility ? shoulder.visibility.toFixed(4) : 'N/A'}`);
                console.log(`L_Elbow (13): x:${elbow.x.toFixed(4)}, y:${elbow.y.toFixed(4)}, z:${elbow.z ? elbow.z.toFixed(4) : 'N/A'}, vis:${elbow.visibility ? elbow.visibility.toFixed(4) : 'N/A'}`);
                console.log(`L_Wrist (15): x:${wrist.x.toFixed(4)}, y:${wrist.y.toFixed(4)}, z:${wrist.z ? wrist.z.toFixed(4) : 'N/A'}, vis:${wrist.visibility ? wrist.visibility.toFixed(4) : 'N/A'}`);
                console.groupEnd();
            }
    
            // === PURE 2D APPROACH ===
            
            // 1. Calculate upper arm vector (shoulder to elbow)
            const upperArmVec = create2DVector(shoulder, elbow);
            
            // 2. Calculate forearm vector (elbow to wrist)
            const forearmVec = create2DVector(elbow, wrist);
    
            const upperArmAngle = angle2D(upperArmVec); // Angle relative to horizontal
            
            // Calculate arm segment lengths for heuristic
            const upperArmLength = vectorMagnitude(upperArmVec);
            const forearmLength = vectorMagnitude(forearmVec);
            const totalArmLength = upperArmLength + forearmLength;
            
            // Add current length to history
            this.armLengthHistory_L.push(totalArmLength);
            if (this.armLengthHistory_L.length > 20) {
                this.armLengthHistory_L.shift(); // Keep only last 20 samples
            }
            
            // Calculate baseline as the maximum length seen (representing full extension sideways)
            const maxArmLength_L = Math.max(...this.armLengthHistory_L);
            
            // Calculate length ratio (current length / baseline length)
            const lengthRatio_L = totalArmLength / maxArmLength_L;
            
            const FORESHORTENING_THRESHOLD = 0.75; // This value might need tuning
            
            let shoulderX_L = 0;
            let isHandsFront_L = false;
            
            const isHorizontal_L = Math.abs(upperArmAngle) < Math.PI/3; // Within 60 degrees of horizontal
            
            if (this.debugMode) {
                console.log(`L_Arm Debug: totalArmLength_L: ${totalArmLength.toFixed(3)}, maxArmLength_L: ${maxArmLength_L.toFixed(3)}, lengthRatio_L: ${lengthRatio_L.toFixed(3)}`);
                console.log(`L_Arm Debug: isHorizontal_L: ${isHorizontal_L}, upperArmAngle: ${(upperArmAngle * 180 / Math.PI).toFixed(1)}°`);
            }

            if (isHorizontal_L && lengthRatio_L < FORESHORTENING_THRESHOLD && this.armLengthHistory_L.length > 10) {
                isHandsFront_L = true;
                shoulderX_L = 1.5; // Hands front, value 1.5
                if (this.debugMode) console.log("L_Arm Debug: isHandsFront_L DETECTED!");
            } else {
                if (this.debugMode) console.log("L_Arm Debug: isHandsFront_L NOT DETECTED.");
                // If not front, check for up/down or sideways
                const absAngle = Math.abs(upperArmAngle);
                if (absAngle > Math.PI/4) { // Arm significantly up or down
                    shoulderX_L = 1.5; // This implies arm is still somewhat "forward" or "up/down" motion
                } else { // Arm relatively horizontal, not detected as front, so assume sideways
                    const t = absAngle / (Math.PI/4); // 0 to 1, as arm moves from horizontal to 45 deg up/down
                    shoulderX_L = t * 1.5; // Interpolate between 0 (sideways) and 1.5 (more forward)
                }
            }
            
            shoulderX_L = smooth('l_shoulder_x', shoulderX_L);
            shoulderX_L = Math.max(-1.832, Math.min(1.919, shoulderX_L)); // Clamp to robot limits

            let shoulderY_L = 0;
            
            // Logic for l_shoulder_y (up/down movement)
            if (isHandsFront_L) {
                shoulderY_L = 0; // When hands are front, y-movement is less relevant, can default to 0
            } else {
                // Map upperArmVec.y (negative for up, positive for down in MediaPipe coords) 
                // to l_shoulder_y (negative for up, positive for down for robot)
                const verticalRatio = upperArmVec.y / upperArmLength; // This will be negative for up, positive for down
                shoulderY_L = verticalRatio * 1.5; // Scale to robot range
            }
            
            shoulderY_L = smooth('l_shoulder_y', shoulderY_L);
            shoulderY_L = Math.max(-2.094, Math.min(2.705, shoulderY_L)); // Clamp to robot limits
    
            let armZ_L = 0; // Currently not mapped from 2D pose, assumed to be 0 (neutral rotation)
            armZ_L = smooth('l_arm_z', armZ_L);
    
            // === LEFT ELBOW Y LOGIC ===
            let elbowY_L = 0;
            const currentElbowAngle_L = angleBetweenVectors(upperArmVec, forearmVec); // Angle at human elbow
            
            // As human elbow angle increases (flexes), robot's l_elbow_y decreases (goes negative)
            // 0.02 is straight, -2.58 is fully bent (from robot limits)
            elbowY_L = (-2.6 / Math.PI) * currentElbowAngle_L + 0.02; // Map 0-PI to 0.02 to -2.58 (approx -2.6 range)

            elbowY_L = smooth('l_elbow_y', elbowY_L);
            elbowY_L = Math.max(-2.58, Math.min(0.02, elbowY_L)); // Clamp to robot's actual limits
    
            // Update robot joints for left hand
            this.viewer?.updateJoint('l_shoulder_x', shoulderX_L);
            this.viewer?.updateJoint('l_shoulder_y', shoulderY_L);
            this.viewer?.updateJoint('l_arm_z', armZ_L);
            this.viewer?.updateJoint('l_elbow_y', elbowY_L);
    
            if (this.debugMode) {
                console.log('2D Mapping Left Hand Results (Smoothed):', {
                    currentElbowAngle: (currentElbowAngle_L * 180 / Math.PI).toFixed(1) + '°',
                    elbowY: elbowY_L.toFixed(3),
                    shoulderX: shoulderX_L.toFixed(3),
                    shoulderY: shoulderY_L.toFixed(3),
                    armZ: armZ_L.toFixed(3)
                });
            }
        }
    
        // --- Right Hand Logic ---
        if (!isValid(poseLandmarks[POSE.RIGHT_SHOULDER]) || 
            !isValid(poseLandmarks[POSE.RIGHT_ELBOW]) || 
            !isValid(poseLandmarks[POSE.RIGHT_WRIST])) {
            if (this.debugMode) console.warn("Invalid right hand landmarks, defaulting to initial pose for right arm.");
            // Use initial pose values as safe defaults if a limb is invalid
            this.viewer?.updateJoint('r_shoulder_x', this.initialRobotPose.r_shoulder_x);
            this.viewer?.updateJoint('r_shoulder_y', this.initialRobotPose.r_shoulder_y);
            this.viewer?.updateJoint('r_arm_z', this.initialRobotPose.r_arm_z);
            this.viewer?.updateJoint('r_elbow_y', this.initialRobotPose.r_elbow_y);
        } else {
            const shoulder_R = poseLandmarks[POSE.RIGHT_SHOULDER];
            const elbow_R = poseLandmarks[POSE.RIGHT_ELBOW];
            const wrist_R = poseLandmarks[POSE.RIGHT_WRIST];

            if (this.debugMode) {
                console.groupCollapsed('Right Arm Debug (Raw Landmarks)');
                console.log(`R_Shoulder (12): x:${shoulder_R.x.toFixed(4)}, y:${shoulder_R.y.toFixed(4)}, z:${shoulder_R.z ? shoulder_R.z.toFixed(4) : 'N/A'}, vis:${shoulder_R.visibility ? shoulder_R.visibility.toFixed(4) : 'N/A'}`);
                console.log(`R_Elbow (14): x:${elbow_R.x.toFixed(4)}, y:${elbow_R.y.toFixed(4)}, z:${elbow_R.z ? elbow_R.z.toFixed(4) : 'N/A'}, vis:${elbow_R.visibility ? elbow_R.visibility.toFixed(4) : 'N/A'}`);
                console.log(`R_Wrist (16): x:${wrist_R.x.toFixed(4)}, y:${wrist_R.y.toFixed(4)}, z:${wrist_R.z ? wrist_R.z.toFixed(4) : 'N/A'}, vis:${wrist_R.visibility ? wrist_R.visibility.toFixed(4) : 'N/A'}`);
                console.groupEnd();
            }
    
            const upperArmVec_R = create2DVector(shoulder_R, elbow_R);
            const forearmVec_R = create2DVector(elbow_R, wrist_R);
    
            const upperArmAngle_R = angle2D(upperArmVec_R); // Angle relative to horizontal
            
            const upperArmLength_R = vectorMagnitude(upperArmVec_R);
            const forearmLength_R = vectorMagnitude(forearmVec_R);
            const totalArmLength_R = upperArmLength_R + forearmLength_R;
            
            this.armLengthHistory_R.push(totalArmLength_R);
            if (this.armLengthHistory_R.length > 20) {
                this.armLengthHistory_R.shift();
            }
            
            const maxArmLength_R = Math.max(...this.armLengthHistory_R);
            const lengthRatio_R = totalArmLength_R / maxArmLength_R;
            
            const FORESHORTENING_THRESHOLD = 0.75; // Can be tuned, potentially different for left/right
            
            let shoulderX_R = 0;
            let isHandsFront_R = false;
            
            const isHorizontal_R = Math.abs(upperArmAngle_R) < Math.PI/3;

            if (this.debugMode) {
                console.log(`R_Arm Debug: totalArmLength_R: ${totalArmLength_R.toFixed(3)}, maxArmLength_R: ${maxArmLength_R.toFixed(3)}, lengthRatio_R: ${lengthRatio_R.toFixed(3)}`);
                console.log(`R_Arm Debug: isHorizontal_R: ${isHorizontal_R}, upperArmAngle_R: ${(upperArmAngle_R * 180 / Math.PI).toFixed(1)}°`);
            }
            
            if (isHorizontal_R && lengthRatio_R < FORESHORTENING_THRESHOLD && this.armLengthHistory_R.length > 10) {
                isHandsFront_R = true;
                shoulderX_R = 1.5; // From table: r_shoulder_x (1.5) for Hands Front
                if (this.debugMode) console.log("R_Arm Debug: isHandsFront_R DETECTED!");
            } else {
                if (this.debugMode) console.log("R_Arm Debug: isHandsFront_R NOT DETECTED.");
                // This block is executed if arm is not detected as "front" by foreshortening
                // For right arm, upperArmAngle_R near +/- PI (180deg) means sideways.
                // Calculate shortest angular distance to either 0 or PI (horizontal axes)
                const angleRelativeToNearestHorizontalAxis = Math.min(
                    Math.abs(upperArmAngle_R),
                    Math.abs(upperArmAngle_R - Math.PI),
                    Math.abs(upperArmAngle_R + Math.PI)
                );

                if (angleRelativeToNearestHorizontalAxis > Math.PI/4) { // Arm significantly up or down relative to horizontal
                    shoulderX_R = 1.5; // This implies arm is still somewhat "forward" or "up/down" motion
                } else { // Arm relatively horizontal (near 0 or +/- PI), not detected as front, so assume sideways
                    const t = angleRelativeToNearestHorizontalAxis / (Math.PI/4); // 0 to 1, as arm moves from horizontal to 45 deg up/down
                    shoulderX_R = t * 1.5; // Interpolate between 0 (sideways) and 1.5 (more forward)
                }
            }
            
            shoulderX_R = smooth('r_shoulder_x', shoulderX_R);
            shoulderX_R = Math.max(-1.919, Math.min(1.832, shoulderX_R)); // Right shoulder_x limits

            let shoulderY_R = 0;
            
            // Logic for r_shoulder_y (up/down movement)
            if (isHandsFront_R) {
                shoulderY_R = 0; // When hands are front, y-movement is less relevant, can default to 0
            } else {
                // Map upperArmVec_R.y (negative for up, positive for down in MediaPipe coords) 
                // to r_shoulder_y (positive for up, negative for down for robot) - inverted from left arm
                const verticalRatio_R = upperArmVec_R.y / upperArmLength_R; // Negative for up, positive for down
                shoulderY_R = -verticalRatio_R * 1.5; // Invert the sign for the right arm's convention
            }
            
            shoulderY_R = smooth('r_shoulder_y', shoulderY_R);
            shoulderY_R = Math.max(-2.705, Math.min(2.094, shoulderY_R)); // Right shoulder_y limits
    
            let armZ_R = 0; // Currently not mapped from 2D pose, assumed to be 0 (neutral rotation)
            armZ_R = smooth('r_arm_z', armZ_R);
    
            // === RIGHT ELBOW Y LOGIC (Already seems consistent with mirrored joint) ===
            let elbowY_R = 0; 
            const currentElbowAngle_R = angleBetweenVectors(upperArmVec_R, forearmVec_R); // Angle at human elbow

            // This mapping means as human elbow angle increases (flexes), robot's r_elbow_y increases.
            // This is consistent with a mirrored joint if l_elbow_y decreases for flexion.
            elbowY_R = (2.6 / Math.PI) * currentElbowAngle_R - 0.02; // Map 0-PI to -0.02 to 2.58 (approx 2.6 range)
            
            elbowY_R = smooth('r_elbow_y', elbowY_R);
            elbowY_R = Math.max(-0.02, Math.min(2.58, elbowY_R)); // Clamp to robot's actual limits
    
            // Update robot joints for right hand
            this.viewer?.updateJoint('r_shoulder_x', shoulderX_R);
            this.viewer?.updateJoint('r_shoulder_y', shoulderY_R);
            this.viewer?.updateJoint('r_arm_z', armZ_R);
            this.viewer?.updateJoint('r_elbow_y', elbowY_R);
    
            if (this.debugMode) {
                console.log('2D Mapping Right Hand Results (Smoothed):', {
                    currentElbowAngle: (currentElbowAngle_R * 180 / Math.PI).toFixed(1) + '°',
                    elbowY: elbowY_R.toFixed(3),
                    shoulderX: shoulderX_R.toFixed(3),
                    shoulderY: shoulderY_R.toFixed(3),
                    armZ: armZ_R.toFixed(3)
                });
            }
        }
    
        // --- Head Logic ---
        if (isValid(poseLandmarks[POSE.NOSE]) &&
            isValid(poseLandmarks[POSE.LEFT_EYE_OUTER]) &&
            isValid(poseLandmarks[POSE.RIGHT_EYE_OUTER]) &&
            isValid(poseLandmarks[POSE.LEFT_EAR]) &&
            isValid(poseLandmarks[POSE.RIGHT_EAR])) {
    
            const nose = poseLandmarks[POSE.NOSE];
            const leftEye = poseLandmarks[POSE.LEFT_EYE_OUTER];
            const rightEye = poseLandmarks[POSE.RIGHT_EYE_OUTER];

            if (this.debugMode) {
                console.groupCollapsed('Head Debug (Raw Landmarks)');
                console.log(`Nose (0): x:${nose.x.toFixed(4)}, y:${nose.y.toFixed(4)}, z:${nose.z ? nose.z.toFixed(4) : 'N/A'}, vis:${nose.visibility ? nose.visibility.toFixed(4) : 'N/A'}`);
                console.log(`L_Eye_Outer (3): x:${leftEye.x.toFixed(4)}, y:${leftEye.y.toFixed(4)}, z:${leftEye.z ? leftEye.z.toFixed(4) : 'N/A'}, vis:${leftEye.visibility ? leftEye.visibility.toFixed(4) : 'N/A'}`);
                console.log(`R_Eye_Outer (6): x:${rightEye.x.toFixed(4)}, y:${rightEye.y.toFixed(4)}, z:${rightEye.z ? rightEye.z.toFixed(4) : 'N/A'}, vis:${rightEye.visibility ? rightEye.visibility.toFixed(4) : 'N/A'}`);
                console.groupEnd();
            }
            
            // --- Calculate head_z (Yaw - left/right rotation) ---
            // Using the horizontal difference between the center of the eyes and the nose
            const eyeMidpointX = (leftEye.x + rightEye.x) / 2;
            const yawMovement = nose.x - eyeMidpointX;
            const yawSensitivity = 15; // *** CALIBRATE THIS *** (e.g., 2 to 10)
            let head_z_val = yawMovement * yawSensitivity;
            
            // Clamp to URDF limits
            head_z_val = Math.max(-1.57079632679, Math.min(1.57079632679, head_z_val));
            head_z_val = smooth('head_z', head_z_val);
            this.viewer?.updateJoint('head_z', head_z_val);
    
            // --- Calculate head_y (Pitch - up/down rotation) ---
            // Using the vertical difference between the nose and the average vertical position of the eyes.
            const eyeMidpointY = (leftEye.y + rightEye.y) / 2;
            const pitchMovement = nose.y - eyeMidpointY;
            
            const pitchSensitivity = 6; // *** CALIBRATE THIS *** (e.g., 2 to 10)
            let head_y_val = -pitchMovement * pitchSensitivity; // Invert the movement
            
            // Clamp to URDF limits
            head_y_val = Math.max(-0.785398163397, Math.min(0.10471975512, head_y_val));
            head_y_val = smooth('head_y', head_y_val);
            this.viewer?.updateJoint('head_y', head_y_val);
    
    
            if (this.debugMode) {
                console.log('2D Mapping Head Results (Smoothed):', {
                    head_z: head_z_val.toFixed(3),
                    head_y: head_y_val.toFixed(3)
                });
            }
    
        } else {
            if (this.debugMode) console.warn("Invalid head landmarks, defaulting head joints to initial pose.");
            this.viewer?.updateJoint('head_z', this.initialRobotPose.head_z);
            this.viewer?.updateJoint('head_y', this.initialRobotPose.head_y);
        }
    }
    
} // End of MediaPipeHandController class