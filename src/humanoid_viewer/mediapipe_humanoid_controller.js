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
                        // --- NEW: Console log the exact landmark points ---
                        console.log('Pose Landmarker Points:', poseLandmarks.map((lm, index) => ({
                            index: index,
                            x: lm.x.toFixed(4),
                            y: lm.y.toFixed(4),
                            z: lm.z ? lm.z.toFixed(4) : 'N/A', // z might be undefined for 2D models
                            visibility: lm.visibility ? lm.visibility.toFixed(4) : 'N/A' // visibility might be undefined
                        })));
                        // --- END NEW ---
                        this.mapLandmarksToRobot(poseLandmarks);
                        this.drawLandmarks(poseLandmarks);
                    } else {
                        this.drawLandmarks([]); // Clear overlay if no pose
                        console.log('No pose landmarks detected.'); // Log when no pose is detected
                    }
                } else {
                    this.drawLandmarks([]); // Clear overlay if video is paused/no new frame
                    // console.log('No new video frame, skipping pose detection.'); // Optional: log if no new frame
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

        // Define connections for the specific body parts (Hands/Arms and Head)
        const connections = [
            // Left Arm
            [11, 13], [13, 15], // Shoulder to Elbow to Wrist
            [15, 17], [17, 19], [19, 21], // Wrist to Pinky, Index, Thumb (simplified connections)
            
            // Right Arm
            [12, 14], [14, 16], // Shoulder to Elbow to Wrist
            [16, 18], [18, 20], [20, 22], // Wrist to Pinky, Index, Thumb (simplified connections)
            
            // Head (simplified connections for visibility)
            [0, 1], [0, 4], // Nose to eyes
            [1, 2], [2, 3], // Left eye connections
            [4, 5], [5, 6], // Right eye connections
            [9, 10] // Mouth (if present and desired)
        ];

        // Define the indices of landmarks to draw (Hands/Arms and Head)
        const landmarksToDrawIndices = new Set([
            0, // Nose
            1, 2, 3, // Left Eye
            4, 5, 6, // Right Eye
            7, 8, // Ears
            9, 10, // Mouth (if present)
            11, 12, // Shoulders
            13, 14, // Elbows
            15, 16, // Wrists
            17, 18, 19, 20, 21, 22 // Hand points
        ]);

        this.overlayCtx.strokeStyle = '#33aaff'; // A vibrant blue for lines
        this.overlayCtx.lineWidth = 5; // Increased line width for connections
        for (const connection of connections) {
            const start = landmarks[connection[0]];
            const end = landmarks[connection[1]];
            
            // Only draw connection if both landmarks are in our desired set
            if (start && end && landmarksToDrawIndices.has(connection[0]) && landmarksToDrawIndices.has(connection[1])) {
                this.overlayCtx.beginPath();
                this.overlayCtx.moveTo(start.x * scaleX, start.y * scaleY);
                this.overlayCtx.lineTo(end.x * scaleX, end.y * scaleY);
                this.overlayCtx.stroke();
            }
        }

        // Draw only specific landmarks (points)
        this.overlayCtx.fillStyle = '#FF4136'; // Red for landmarks
        this.overlayCtx.strokeStyle = '#FFFFFF'; // White border
        this.overlayCtx.lineWidth = 2; // Increased line width for landmark borders
        for (let i = 0; i < landmarks.length; i++) {
            if (landmarksToDrawIndices.has(i)) { // Only draw if index is in our allowed set
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


    // ... (previous code remains the same until mapLandmarksToRobot)

    // ... (previous code remains the same until mapLandmarksToRobot)

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
            if (isNaN(value)) return this.previousJointValues[jointName] || (value || 0); // Handle initial NaN and provide a default
            const prev = this.previousJointValues[jointName] ?? value;
            const smoothed = prev * (1 - alpha) + value * alpha;
            this.previousJointValues[jointName] = smoothed;
            return smoothed;
        };
    
        // Landmark validation
        const isValid = (landmark) => landmark && 
            (landmark.visibility === undefined || landmark.visibility > 0.5);
    
        // --- Left Hand Logic ---
        if (!isValid(poseLandmarks[POSE.LEFT_SHOULDER]) || 
            !isValid(poseLandmarks[POSE.LEFT_ELBOW]) || 
            !isValid(poseLandmarks[POSE.LEFT_WRIST])) {
            console.warn("Invalid left hand landmarks, using safe 2D position");
            // Safe 2D position: arm horizontal
            this.viewer?.updateJoint('l_shoulder_x', 0);       // Horizontal
            this.viewer?.updateJoint('l_shoulder_y', 0);       // No front/back
            this.viewer?.updateJoint('l_arm_z', 0);             // No rotation
            this.viewer?.updateJoint('l_elbow_y', 0.02); // Set to default/straight for left elbow
        } else {
            const shoulder = poseLandmarks[POSE.LEFT_SHOULDER];
            const elbow = poseLandmarks[POSE.LEFT_ELBOW];
            const wrist = poseLandmarks[POSE.LEFT_WRIST];
    
            // === PURE 2D APPROACH ===
            
            // 1. Calculate upper arm vector (shoulder to elbow)
            const upperArmVec = create2DVector(shoulder, elbow);
            
            // 2. Calculate forearm vector (elbow to wrist)
            const forearmVec = create2DVector(elbow, wrist);
    
            const upperArmAngle = angle2D(upperArmVec);
            
            // Calculate arm segment lengths for heuristic
            const upperArmLength = vectorMagnitude(upperArmVec);
            const forearmLength = vectorMagnitude(forearmVec);
            const totalArmLength = upperArmLength + forearmLength;
            
            // Store baseline arm length (use a rolling average for stability)
            if (!this.armLengthHistory_L) {
                this.armLengthHistory_L = [];
            }
            
            // Add current length to history
            this.armLengthHistory_L.push(totalArmLength);
            if (this.armLengthHistory_L.length > 20) {
                this.armLengthHistory_L.shift(); // Keep only last 20 samples
            }
            
            // Calculate baseline as the maximum length seen (representing full extension sideways)
            const maxArmLength_L = Math.max(...this.armLengthHistory_L);
            
            // Calculate length ratio (current length / baseline length)
            const lengthRatio_L = totalArmLength / maxArmLength_L;
            
            const FORESHORTENING_THRESHOLD = 0.75;
            
            let shoulderX_L = 0;
            let isHandsFront_L = false;
            
            const isHorizontal_L = Math.abs(upperArmAngle) < Math.PI/3;
            
            if (isHorizontal_L && lengthRatio_L < FORESHORTENING_THRESHOLD && this.armLengthHistory_L.length > 10) {
                isHandsFront_L = true;
                shoulderX_L = 1.5;
            } else if (upperArmAngle < -Math.PI/4) {
                shoulderX_L = 1.5;
            } else if (upperArmAngle > Math.PI/4) { 
                shoulderX_L = 1.5;
            } else {
                shoulderX_L = 0;
            }
            
            if (!isHandsFront_L) {
                const absAngle = Math.abs(upperArmAngle);
                if (absAngle > Math.PI/4) {
                    shoulderX_L = 1.5;
                } else {
                    const t = absAngle / (Math.PI/4);
                    shoulderX_L = t * 1.5;
                }
            }
            
            shoulderX_L = smooth('l_shoulder_x', shoulderX_L);
            shoulderX_L = Math.max(-1.832, Math.min(1.919, shoulderX_L));
    
            let shoulderY_L = 0;
            
            if (isHandsFront_L) {
                shoulderY_L = 0;
            } else if (upperArmAngle < -Math.PI/4) {
                shoulderY_L = -1.5;
            } else if (upperArmAngle > Math.PI/4) {
                shoulderY_L = 1.5;
            } else {
                shoulderY_L = 0;
            }
            
            if (!isHandsFront_L) {
                if (upperArmAngle < -Math.PI/4) {
                    shoulderY_L = -1.5;
                } else if (upperArmAngle > Math.PI/4) {
                    shoulderY_L = 1.5;
                } else {
                    const normalizedAngle = upperArmAngle / (Math.PI/4);
                    shoulderY_L = normalizedAngle * 1.5;
                }
            }
            
            shoulderY_L = smooth('l_shoulder_y', shoulderY_L);
            shoulderY_L = Math.max(-2.094, Math.min(2.705, shoulderY_L));
    
            let armZ_L = 0;
            armZ_L = smooth('l_arm_z', armZ_L);
    
            // === LEFT ELBOW Y LOGIC (Corrected for inversion) ===
            let elbowY_L = 0;
            const currentElbowAngle_L = angleBetweenVectors(upperArmVec, forearmVec); // Angle at human elbow
            
            // Robot l_elbow_y: -2.58 (bent) to 0.02 (straight)
            // Human Angle: PI (straight) to 0 (bent)
            // We need to map:
            // Human PI (straight) -> Robot -2.58 (bent)
            // Human 0 (bent) -> Robot 0.02 (straight)

            // Calculate slope and intercept for this inverted mapping
            // Points: (Math.PI, -2.58) and (0, 0.02)
            // Slope m = (0.02 - (-2.58)) / (0 - Math.PI) = 2.6 / -Math.PI = -2.6 / Math.PI
            // Intercept b = 0.02 (when currentElbowAngle_L is 0)
            
            elbowY_L = (-2.6 / Math.PI) * currentElbowAngle_L + 0.02;

            elbowY_L = smooth('l_elbow_y', elbowY_L);
            elbowY_L = Math.max(-2.58, Math.min(0.02, elbowY_L)); // Clamp to robot's actual limits
    
            // Update robot joints for left hand
            this.viewer?.updateJoint('l_shoulder_x', shoulderX_L);
            this.viewer?.updateJoint('l_shoulder_y', shoulderY_L);
            this.viewer?.updateJoint('l_arm_z', armZ_L);
            this.viewer?.updateJoint('l_elbow_y', elbowY_L);
    
            if (this.debugMode) {
                console.log('2D Mapping Left Hand:', {
                    upperArmAngle: (upperArmAngle * 180 / Math.PI).toFixed(1) + '°',
                    totalArmLength: totalArmLength.toFixed(3),
                    maxArmLength: (this.armLengthHistory_L ? Math.max(...this.armLengthHistory_L).toFixed(3) : 'N/A'),
                    lengthRatio: lengthRatio_L.toFixed(3),
                    isHandsFront: isHandsFront_L,
                    shoulderX: shoulderX_L.toFixed(3),
                    shoulderY: shoulderY_L.toFixed(3),
                    armZ: armZ_L.toFixed(3),
                    elbowY: elbowY_L.toFixed(3),
                    currentElbowAngle: (currentElbowAngle_L * 180 / Math.PI).toFixed(1) + '°'
                });
            }
        }
    
        // --- Right Hand Logic ---
        if (!isValid(poseLandmarks[POSE.RIGHT_SHOULDER]) || 
            !isValid(poseLandmarks[POSE.RIGHT_ELBOW]) || 
            !isValid(poseLandmarks[POSE.RIGHT_WRIST])) {
            console.warn("Invalid right hand landmarks, using safe 2D position");
            // Safe 2D position: arm horizontal
            this.viewer?.updateJoint('r_shoulder_x', 0);       // Horizontal
            this.viewer?.updateJoint('r_shoulder_y', 0);       // No front/back
            this.viewer?.updateJoint('r_arm_z', 0);             // No rotation
            this.viewer?.updateJoint('r_elbow_y', -0.02); // Set to default/straight for right elbow
        } else {
            const shoulder_R = poseLandmarks[POSE.RIGHT_SHOULDER];
            const elbow_R = poseLandmarks[POSE.RIGHT_ELBOW];
            const wrist_R = poseLandmarks[POSE.RIGHT_WRIST];
    
            const upperArmVec_R = create2DVector(shoulder_R, elbow_R);
            const forearmVec_R = create2DVector(elbow_R, wrist_R);
    
            const upperArmAngle_R = angle2D(upperArmVec_R);
            
            const upperArmLength_R = vectorMagnitude(upperArmVec_R);
            const forearmLength_R = vectorMagnitude(forearmVec_R);
            const totalArmLength_R = upperArmLength_R + forearmLength_R;
            
            if (!this.armLengthHistory_R) {
                this.armLengthHistory_R = [];
            }
            
            this.armLengthHistory_R.push(totalArmLength_R);
            if (this.armLengthHistory_R.length > 20) {
                this.armLengthHistory_R.shift();
            }
            
            const maxArmLength_R = Math.max(...this.armLengthHistory_R);
            const lengthRatio_R = totalArmLength_R / maxArmLength_R;
            
            const FORESHORTENING_THRESHOLD = 0.75;
            
            let shoulderX_R = 0;
            let isHandsFront_R = false;
            
            const isHorizontal_R = Math.abs(upperArmAngle_R) < Math.PI/3;
            
            if (isHorizontal_R && lengthRatio_R < FORESHORTENING_THRESHOLD && this.armLengthHistory_R.length > 10) {
                isHandsFront_R = true;
                shoulderX_R = 1.5; // From table: r_shoulder_x (1.5) for Hands Front
            } else if (upperArmAngle_R < -Math.PI/4) { // Hands up
                shoulderX_R = 1.5; // From table: r_shoulder_x (1.5) for Hands Up
            } else if (upperArmAngle_R > Math.PI/4) { // Hands down
                shoulderX_R = 1.5; // From table: r_shoulder_x (1.5) for Hands Down
            } else { // Hands sideways
                shoulderX_R = 0; // From table: r_shoulder_x (0) for Hands Sideways
            }
    
            if (!isHandsFront_R) {
                const absAngle = Math.abs(upperArmAngle_R);
                if (absAngle > Math.PI/4) {
                    shoulderX_R = 1.5;
                } else {
                    const t = absAngle / (Math.PI/4);
                    shoulderX_R = t * 1.5;
                }
            }
            
            shoulderX_R = smooth('r_shoulder_x', shoulderX_R);
            shoulderX_R = Math.max(-1.919, Math.min(1.832, shoulderX_R)); // Right shoulder_x limits
    
            let shoulderY_R = 0;
            
            if (isHandsFront_R) {
                shoulderY_R = 0; // From table: r_shoulder_y (0) for Hands Front
            } else if (upperArmAngle_R < -Math.PI/4) { // Hands up
                shoulderY_R = 1.5; // From table: r_shoulder_y (1.5) for Hands Up
            } else if (upperArmAngle_R > Math.PI/4) { // Hands down
                shoulderY_R = -1.5; // From table: r_shoulder_y (-1.5) for Hands Down
            } else { // Hands sideways
                shoulderY_R = 0; // From table: r_shoulder_y (0) for Hands Sideways
            }
    
            if (!isHandsFront_R) {
                if (upperArmAngle_R < -Math.PI/4) { // Up
                    shoulderY_R = 1.5;
                } else if (upperArmAngle_R > Math.PI/4) { // Down
                    shoulderY_R = -1.5;
                } else { // Sideways interpolation
                    const normalizedAngle = upperArmAngle_R / (Math.PI/4); // -1 to +1
                    shoulderY_R = normalizedAngle * -1.5; // Invert to match r_shoulder_y movement
                }
            }
            
            shoulderY_R = smooth('r_shoulder_y', shoulderY_R);
            shoulderY_R = Math.max(-2.705, Math.min(2.094, shoulderY_R)); // Right shoulder_y limits
    
            let armZ_R = 0;
            armZ_R = smooth('r_arm_z', armZ_R);
    
            // === RIGHT ELBOW Y LOGIC (Corrected for inversion) ===
            let elbowY_R = 0; 
            const currentElbowAngle_R = angleBetweenVectors(upperArmVec_R, forearmVec_R); // Angle at human elbow

            // Robot r_elbow_y: -0.02 (straight) to 2.58 (bent)
            // Human Angle: PI (straight) to 0 (bent)
            // We need to map:
            // Human PI (straight) -> Robot 2.58 (bent)
            // Human 0 (bent) -> Robot -0.02 (straight)

            // Calculate slope and intercept for this inverted mapping
            // Points: (Math.PI, 2.58) and (0, -0.02)
            // Slope m = (-0.02 - 2.58) / (0 - Math.PI) = -2.6 / -Math.PI = 2.6 / Math.PI
            // Intercept b = -0.02 (when currentElbowAngle_R is 0)
            
            elbowY_R = (2.6 / Math.PI) * currentElbowAngle_R - 0.02;
            
            elbowY_R = smooth('r_elbow_y', elbowY_R);
            elbowY_R = Math.max(-0.02, Math.min(2.58, elbowY_R)); // Clamp to robot's actual limits
    
            // Update robot joints for right hand
            this.viewer?.updateJoint('r_shoulder_x', shoulderX_R);
            this.viewer?.updateJoint('r_shoulder_y', shoulderY_R);
            this.viewer?.updateJoint('r_arm_z', armZ_R);
            this.viewer?.updateJoint('r_elbow_y', elbowY_R);
    
            if (this.debugMode) {
                console.log('2D Mapping Right Hand:', {
                    upperArmAngle: (upperArmAngle_R * 180 / Math.PI).toFixed(1) + '°',
                    totalArmLength: totalArmLength_R.toFixed(3),
                    maxArmLength: (this.armLengthHistory_R ? Math.max(...this.armLengthHistory_R).toFixed(3) : 'N/A'),
                    lengthRatio: lengthRatio_R.toFixed(3),
                    isHandsFront: isHandsFront_R,
                    shoulderX: shoulderX_R.toFixed(3),
                    shoulderY: shoulderY_R.toFixed(3),
                    armZ: armZ_R.toFixed(3),
                    elbowY: elbowY_R.toFixed(3)
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
            // Ears are checked for validity but not directly used in the calculations below.
            // const leftEar = poseLandmarks[POSE.LEFT_EAR];
            // const rightEar = poseLandmarks[POSE.RIGHT_EAR];
    
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
            
            const pitchSensitivity = 15; // *** CALIBRATE THIS *** (e.g., 2 to 10)
            let head_y_val = -pitchMovement * pitchSensitivity; // Invert the movement
            
            // Clamp to URDF limits
            head_y_val = Math.max(-0.785398163397, Math.min(0.10471975512, head_y_val));
            head_y_val = smooth('head_y', head_y_val);
            this.viewer?.updateJoint('head_y', head_y_val);
    
    
            if (this.debugMode) {
                console.log('2D Mapping Head:', {
                    head_z: head_z_val.toFixed(3),
                    head_y: head_y_val.toFixed(3)
                });
            }
    
        } else {
            console.warn("Invalid head landmarks, defaulting head joints to 0.");
            this.viewer?.updateJoint('head_z', 0);
            this.viewer?.updateJoint('head_y', 0);
        }
    }
    
// ... (rest of the code remains the same)
    
// ... (rest of the code remains the same)
    // Additional helper: Detect specific 2D poses (no changes here as it's a generic pose detector)
    detect2DPose(poseLandmarks) {
        const shoulder = poseLandmarks[11];
        const elbow = poseLandmarks[13];
        const wrist = poseLandmarks[15];
        
        if (!shoulder || !elbow || !wrist) return "unknown";
        
        const upperArmVec = { x: elbow.x - shoulder.x, y: elbow.y - shoulder.y };
        const forearmVec = { x: wrist.x - elbow.x, y: wrist.y - elbow.y };
        const upperArmAngle = Math.atan2(upperArmVec.y, upperArmVec.x);
        
        const upperArmLength = Math.hypot(upperArmVec.x, upperArmVec.y);
        const forearmLength = Math.hypot(forearmVec.x, forearmVec.y);
        const totalArmLength = upperArmLength + forearmLength;
        
        const estimatedBaseline = totalArmLength / 0.85;
        const lengthRatio = totalArmLength / estimatedBaseline;
        
        const angleDeg = upperArmAngle * 180 / Math.PI;
        
        const isHorizontal = Math.abs(angleDeg) < 45;
        const isShortened = lengthRatio < 0.85;
        
        if (isHorizontal && isShortened) return "hands_front";
        else if (angleDeg < -45) return "hands_up";
        else if (angleDeg > 45) return "hands_down";
        else if (Math.abs(angleDeg) < 15) return "hands_sideways";
        else return "transitioning";
    }
}