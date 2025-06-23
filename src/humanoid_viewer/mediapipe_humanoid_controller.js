// mediapipe_pose_controller.js

import { PoseLandmarker, FilesetResolver } from "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision/vision_bundle.js";

export class MediaPipeHandController { // This should ideally be MediaPipePoseController
    constructor(viewerInstance, videoElement) {
        this.viewer = viewerInstance;
        this.video = videoElement;
        this.poseLandmarker = null;
        this.runningMode = "VIDEO";
        this.webcamInitialized = false;
        this.lastVideoTime = -1;

        // Store previous joint values for smoothing
        this.previousJointValues = {};

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
            numPoses: 1
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
                        this.mapLandmarksToRobot(poseLandmarks);
                        this.drawLandmarks(poseLandmarks);
                    } else {
                        this.drawLandmarks([]); // Clear overlay if no pose
                    }
                } else {
                    this.drawLandmarks([]); // Clear overlay if video is paused/no new frame
                }
            } catch (error) {
                console.error("Pose detection error:", error);
            }
        }
        requestAnimationFrame(this.detectPosesInRealTime.bind(this));
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

        // Define connections for the pose skeleton
        const connections = [
            // Torso
            [11, 12], // Shoulders
            [23, 24], // Hips
            [11, 23], [12, 24], // Left and Right torso connections

            // Left Arm
            [11, 13], [13, 15], // Shoulder to Elbow to Wrist
            [15, 17], [17, 19], [19, 21], // Wrist to Pinky, Index, Thumb (simplified connections)
            
            // Right Arm
            [12, 14], [14, 16], // Shoulder to Elbow to Wrist
            [16, 18], [18, 20], [20, 22], // Wrist to Pinky, Index, Thumb (simplified connections)
            
            // Left Leg
            [23, 25], [25, 27], // Hip to Knee to Ankle
            [27, 29], [29, 31], [31, 27], // Ankle to Heel, Heel to Foot Index, Foot Index to Ankle
            
            // Right Leg
            [24, 26], [26, 28], // Hip to Knee to Ankle
            [28, 30], [30, 32], [32, 28], // Ankle to Heel, Heel to Foot Index, Foot Index to Ankle

            // Face (if included in the heavy model and you wish to draw them)
            [0, 1], [0, 4], // Nose to eyes
            [1, 2], [2, 3], // Left eye connections
            [4, 5], [5, 6], // Right eye connections
            [9, 10] // Mouth
        ];

        this.overlayCtx.strokeStyle = '#33aaff'; // A vibrant blue for lines
        this.overlayCtx.lineWidth = 5; // Increased line width for connections
        for (const connection of connections) {
            const start = landmarks[connection[0]];
            const end = landmarks[connection[1]];
            
            if (start && end) {
                this.overlayCtx.beginPath();
                this.overlayCtx.moveTo(start.x * scaleX, start.y * scaleY);
                this.overlayCtx.lineTo(end.x * scaleX, end.y * scaleY);
                this.overlayCtx.stroke();
            }
        }

        // Draw landmarks (points)
        this.overlayCtx.fillStyle = '#FF4136'; // Red for landmarks
        this.overlayCtx.strokeStyle = '#FFFFFF'; // White border
        this.overlayCtx.lineWidth = 2; // Increased line width for landmark borders
        for (let i = 0; i < landmarks.length; i++) {
            const x = landmarks[i].x * scaleX;
            const y = landmarks[i].y * scaleY;
            
            this.overlayCtx.beginPath();
            this.overlayCtx.arc(x, y, 7, 0, 2 * Math.PI); // Increased radius for landmarks
            this.overlayCtx.fill();
            this.overlayCtx.stroke(); // Draw border
        }

        // Restore the canvas state
        this.overlayCtx.restore();
    }


    mapLandmarksToRobot(poseLandmarks) {
        // MediaPipe Pose Landmark indices (pose_landmarker_heavy model)
        const POSE_LANDMARKS = {
            NOSE: 0,
            LEFT_EYE_INNER: 1, LEFT_EYE: 2, LEFT_EYE_OUTER: 3,
            RIGHT_EYE_INNER: 4, RIGHT_EYE: 5, RIGHT_EYE_OUTER: 6,
            LEFT_EAR: 7, RIGHT_EAR: 8,
            MOUTH_LEFT: 9, MOUTH_RIGHT: 10,
            LEFT_SHOULDER: 11, RIGHT_SHOULDER: 12,
            LEFT_ELBOW: 13, RIGHT_ELBOW: 14,
            LEFT_WRIST: 15, RIGHT_WRIST: 16,
            LEFT_PINKY: 17, RIGHT_PINKY: 18,
            LEFT_INDEX: 19, RIGHT_INDEX: 20,
            LEFT_THUMB: 21, RIGHT_THUMB: 22,
            LEFT_HIP: 23, RIGHT_HIP: 24,
            LEFT_KNEE: 25, RIGHT_KNEE: 26,
            LEFT_ANKLE: 27, RIGHT_ANKLE: 28,
            LEFT_HEEL: 29, RIGHT_HEEL: 30,
            LEFT_FOOT_INDEX: 31, RIGHT_FOOT_INDEX: 32
        };
    
        // Helper function to create a 3D vector
        const createVector = (p1, p2) => ({
            x: p2.x - p1.x,
            y: p2.y - p1.y,
            z: (p2.z || 0) - (p1.z || 0)
        });

        // Helper functions for angle calculations
        const calculateAngleBetweenVectors = (v1, v2) => {
            const dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
            const mag1 = Math.sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
            const mag2 = Math.sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
            
            if (mag1 === 0 || mag2 === 0) return 0;
            
            return Math.acos(Math.max(-1, Math.min(1, dot / (mag1 * mag2))));
        };
    
        const clampAngle = (angle, min, max) => {
            return Math.max(min, Math.min(max, angle));
        };
    
        // Smoothing function to prevent jittery movements
        const smoothAngle = (current, target, smoothingFactor = 0.2) => { // Increased smoothingFactor for smoother motion
            if (current === undefined || current === null) return target;
            return current + (target - current) * smoothingFactor;
        };
    
        // Extract key landmarks (only those needed for arm control)
        const landmarks = {
            leftShoulder: poseLandmarks[POSE_LANDMARKS.LEFT_SHOULDER],
            rightShoulder: poseLandmarks[POSE_LANDMARKS.RIGHT_SHOULDER],
            leftElbow: poseLandmarks[POSE_LANDMARKS.LEFT_ELBOW],
            rightElbow: poseLandmarks[POSE_LANDMARKS.RIGHT_ELBOW],
            leftWrist: poseLandmarks[POSE_LANDMARKS.LEFT_WRIST],
            rightWrist: poseLandmarks[POSE_LANDMARKS.RIGHT_WRIST],
        };
    
        // Helper function to update joint with smoothing and clamping
        const updateJointSmooth = (jointName, targetValue, limits = null) => {
            let clampedValue = targetValue;
            if (limits) {
                clampedValue = clampAngle(targetValue, limits.min, limits.max);
            }
            
            const smoothedValue = smoothAngle(this.previousJointValues[jointName], clampedValue);
            this.previousJointValues[jointName] = smoothedValue;
            
            if (this.viewer && this.viewer.updateJoint) {
                this.viewer.updateJoint(jointName, smoothedValue);
            }
            return smoothedValue;
        };

        // Separate function to void all non-arm joints for clarity and robustness
        this.voidAllNonArmJoints = () => {
            const allRobotJoints = [
                'head_y', 'head_z',
                'abs_y', 'abs_x', 'abs_z', 'bust_y', 'bust_x',
                'l_arm_z', 'r_arm_z', // These are now managed specifically below for 'no movement'
                'l_hip_y', 'l_hip_x', 'l_knee_y', 'l_ankle_y', 
                'r_hip_y', 'r_hip_x', 'r_knee_y', 'r_ankle_y'
            ];
            allRobotJoints.forEach(jointName => {
                // Ensure the 'no movement' joints are explicitly set to a tight zero range
                if (jointName === 'l_arm_z' || jointName === 'r_arm_z') {
                    updateJointSmooth(jointName, 0, { min: -0.001, max: 0.001 }); // Very small range for 'no movement'
                } else {
                    updateJointSmooth(jointName, 0, { min: -0.01, max: 0.01 }); // Set to near zero with tiny range for other voided joints
                }
            });
        };

        // --- VOID ALL NON-ARM JOINTS FIRST ---
        this.voidAllNonArmJoints();
        
        // Check if all required arm landmarks are present. If not, don't update active arm joints.
        const requiredArmLandmarks = [
            landmarks.leftShoulder, landmarks.rightShoulder,
            landmarks.leftElbow, landmarks.rightElbow,
            landmarks.leftWrist, landmarks.rightWrist
        ];

        if (requiredArmLandmarks.some(landmark => !landmark)) {
            console.warn('Some required arm landmarks are missing, skipping active arm joint updates.');
            return; // Exit if essential arm landmarks are missing
        }


        // --- ARM JOINTS (ACTIVE CONTROL) ---

        // Left Arm
        // l_shoulder_y (shoulder raise/lower/pitch) - angle of upper arm relative to torso.
        // Range: -1.5 to 1.5
        const leftUpperArm = createVector(landmarks.leftShoulder, landmarks.leftElbow);
        // Estimate angle using y and z components relative to a 'straight out' pose
        // Adjust the offset and scale to fit the -1.5 to 1.5 range
        let leftShoulderYAngle = Math.atan2(leftUpperArm.y, leftUpperArm.z); 
        // Initial tuning: Math.PI/2 offset assumes neutral arm is along Z.
        // We need to re-map this to the -1.5 to 1.5 range.
        // Example: If straight down is ~0 and straight up is ~PI, and we want -1.5 to 1.5.
        // A direct mapping might be: scaled_angle = (raw_angle / Math.PI) * 1.5;
        // However, a simple linear mapping from a segment of atan2's output to [-1.5, 1.5]
        // or trial-and-error with offset and multiplier is common.
        // Let's try to map human arm's vertical movement to robot's shoulder_y.
        // Assuming human arm starts 'down' (shoulder_y close to 0 or negative), and moves up.
        // The angle needs to be inverted and scaled.
        
        // Normalize the Y-coordinate of the elbow relative to the shoulder
        // Higher Y means arm is raised. Lower Y means arm is down.
        // The Y coordinate goes from 0 (bottom of screen) to 1 (top of screen).
        // Let's use the difference in Y-coordinates between elbow and shoulder.
        // This will give a value typically between -1 and 1.
        const leftArmDeltaY = (landmarks.leftElbow.y - landmarks.leftShoulder.y);
        // Map this delta_y to the robot's joint range (-1.5 to 1.5).
        // A simple linear mapping: input_range_start -> -1.5, input_range_end -> 1.5
        // Let's assume leftArmDeltaY ranges from approx -0.5 (arm down) to 0.5 (arm up).
        // mapped_value = ( (input_value - input_min) / (input_max - input_min) ) * (output_max - output_min) + output_min
        // For -0.5 to 0.5 mapped to -1.5 to 1.5:
        leftShoulderYAngle = ((leftArmDeltaY + 0.5) / 1.0) * (1.5 - (-1.5)) + (-1.5);
        // This scaling factor (1.0 for the divisor) and offset (0.5 for deltaY) will need fine-tuning.
        // A common approach is to find what leftArmDeltaY value corresponds to your desired -1.5 and 1.5 robot joint angles.
        updateJointSmooth('l_shoulder_y', leftShoulderYAngle, { min: -1.5, max: 1.5 });


        // l_shoulder_x (shoulder forward/backward/roll) - based on Z-depth of elbow relative to shoulder.
        // Range: 0 to 1.5
        // Positive Z means closer to camera (less depth). Negative Z means further away.
        // Human arm moving forward, elbow Z relative to shoulder Z should increase (become less negative or more positive).
        // The original code had: (landmarks.leftElbow.z - landmarks.leftShoulder.z) * 5
        const leftArmDeltaZ = (landmarks.leftElbow.z - landmarks.leftShoulder.z);
        // Map leftArmDeltaZ to 0 to 1.5. Assume arm at side (neutral Z diff) maps to 0.
        // Moving arm forward (positive deltaZ) maps to increasing angle up to 1.5.
        // Fine-tune the multiplier and offset.
        let leftShoulderXAngle = leftArmDeltaZ * 3; // Initial guess for sensitivity
        leftShoulderXAngle = Math.max(0, leftShoulderXAngle); // Ensure it's not negative
        // If arm pulled back, it might go below 0. We want to clamp it.
        // You'll need to observe the Z values to determine a good scaling and offset.
        // Example: If arm at side (neutral) is deltaZ ~ 0, and arm fully forward is deltaZ ~ 0.2
        // Then (deltaZ / 0.2) * 1.5
        updateJointSmooth('l_shoulder_x', leftShoulderXAngle, { min: 0, max: 1.5 });

        // l_arm_z (no movement)
        updateJointSmooth('l_arm_z', 0, { min: -0.001, max: 0.001 });

        // l_elbow_y (elbow bend/pitch) - angle at elbow joint. Full range.
        const leftForearm = createVector(landmarks.leftElbow, landmarks.leftWrist);
        const leftElbowAngle = calculateAngleBetweenVectors(leftUpperArm, leftForearm); // Angle between upper arm and forearm
        // Poppy's elbow_y: lower="-2.58308729295" upper="0.0174532925199"
        // Math.PI - leftElbowAngle is a common way to invert the angle for a typical human-robot elbow.
        // The limits already enforce the "full" range.
        updateJointSmooth('l_elbow_y', Math.PI - leftElbowAngle, { min: -2.58308729295, max: 0.0174532925199 });


        // Right Arm (similar logic to left arm, with adjusted min/max for symmetry)
        const rightUpperArm = createVector(landmarks.rightShoulder, landmarks.rightElbow);

        // r_shoulder_y (Range: -1.5 to 1.5)
        const rightArmDeltaY = (landmarks.rightElbow.y - landmarks.rightShoulder.y);
        let rightShoulderYAngle = ((rightArmDeltaY + 0.5) / 1.0) * (1.5 - (-1.5)) + (-1.5);
        // Note the URDF has axis xyz="-1 0 0" for r_shoulder_y, implying a potential sign inversion for symmetry
        updateJointSmooth('r_shoulder_y', -rightShoulderYAngle, { min: -1.5, max: 1.5 });

        // r_shoulder_x (Range: 0 to 1.5)
        const rightArmDeltaZ = (landmarks.rightElbow.z - landmarks.rightShoulder.z);
        let rightShoulderXAngle = rightArmDeltaZ * 3;
        rightShoulderXAngle = Math.max(0, rightShoulderXAngle);
        // Note the URDF has axis xyz="0 0 -1" for r_shoulder_x, implying potential sign inversion.
        updateJointSmooth('r_shoulder_x', -rightShoulderXAngle, { min: 0, max: 1.5 });


        // r_arm_z (no movement)
        updateJointSmooth('r_arm_z', 0, { min: -0.001, max: 0.001 });

        // r_elbow_y (full range)
        const rightForearm = createVector(landmarks.rightElbow, landmarks.rightWrist);
        const rightElbowAngle = calculateAngleBetweenVectors(rightUpperArm, rightForearm);
        // Poppy's r_elbow_y: lower="-0.0174532925199" upper="2.58308729295"
        // Note the URDF has axis xyz="-1 0 0" for r_elbow_y, implying potential sign inversion for symmetry.
        updateJointSmooth('r_elbow_y', -(Math.PI - rightElbowAngle), { min: -0.0174532925199, max: 2.58308729295 });

        // Optional: Log joint updates for debugging (reduced frequency)
        if (Math.random() < 0.05) { // Only log 5% of the time to reduce console spam
            console.log('Joint updates applied:', {
                l_shoulder_y: this.previousJointValues.l_shoulder_y?.toFixed(3),
                l_shoulder_x: this.previousJointValues.l_shoulder_x?.toFixed(3),
                l_elbow_y: this.previousJointValues.l_elbow_y?.toFixed(3),
                r_shoulder_y: this.previousJointValues.r_shoulder_y?.toFixed(3),
                r_shoulder_x: this.previousJointValues.r_shoulder_x?.toFixed(3),
                r_elbow_y: this.previousJointValues.r_elbow_y?.toFixed(3),
                l_arm_z: this.previousJointValues.l_arm_z?.toFixed(3),
                r_arm_z: this.previousJointValues.r_arm_z?.toFixed(3),
                // Show a few voided ones to confirm they are 0
                l_hip_y: this.previousJointValues.l_hip_y?.toFixed(3),
                head_y: this.previousJointValues.head_y?.toFixed(3)
            });
        }
    }
}