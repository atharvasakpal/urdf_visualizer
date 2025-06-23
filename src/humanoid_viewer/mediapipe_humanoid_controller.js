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
        const POSE = {
            LEFT_SHOULDER: 11,
            LEFT_ELBOW: 13,
            LEFT_WRIST: 15,
            RIGHT_SHOULDER: 12
        };
    
        // Helper functions for 2D operations only
        const create2DVector = (p1, p2) => ({
            x: p2.x - p1.x,
            y: p2.y - p1.y
        });
    
        const angle2D = (vec) => Math.atan2(vec.y, vec.x);
        
        const vectorMagnitude = (vec) => Math.hypot(vec.x, vec.y);
    
        // Smoothing
        if (!this.previousJointValues) this.previousJointValues = {};
        const smooth = (jointName, value, alpha = 0.2) => {
            if (isNaN(value)) return this.previousJointValues[jointName] || 0;
            const prev = this.previousJointValues[jointName] ?? value;
            const smoothed = prev * (1 - alpha) + value * alpha;
            this.previousJointValues[jointName] = smoothed;
            return smoothed;
        };
    
        // Landmark validation
        const isValid = (landmark) => landmark && 
            (landmark.visibility === undefined || landmark.visibility > 0.5);
    
        if (!isValid(poseLandmarks[POSE.LEFT_SHOULDER]) || 
            !isValid(poseLandmarks[POSE.LEFT_ELBOW]) || 
            !isValid(poseLandmarks[POSE.LEFT_WRIST])) {
            console.warn("Invalid landmarks, using safe 2D position");
            // Safe 2D position: arm horizontal
            this.viewer?.updateJoint('l_shoulder_x', 0);      // Horizontal
            this.viewer?.updateJoint('l_shoulder_y', 0);      // No front/back
            this.viewer?.updateJoint('l_arm_z', 0);           // No rotation
            this.viewer?.updateJoint('l_elbow_y', 0);         // Set to 0
            return;
        }
    
        const shoulder = poseLandmarks[POSE.LEFT_SHOULDER];
        const elbow = poseLandmarks[POSE.LEFT_ELBOW];
        const wrist = poseLandmarks[POSE.LEFT_WRIST];
    
        // === PURE 2D APPROACH ===
        
        // 1. Calculate upper arm vector (shoulder to elbow)
        const upperArmVec = create2DVector(shoulder, elbow);
        
        // 2. Calculate forearm vector (elbow to wrist)
        const forearmVec = create2DVector(elbow, wrist);
    
        // === SHOULDER X (Up/Down movement) ===
        // Based on your tested mapping:
        // Hands up: shoulder_x = 1.5
        // Hands down: shoulder_x = 1.5  
        // Hands front: shoulder_x = 1.5
        // Hands sideways: shoulder_x = 0 (ORIGINAL POSITION)
        
        const upperArmAngle = angle2D(upperArmVec);
        const forearmAngle = angle2D(forearmVec);
        
        // Calculate arm segment lengths for heuristic
        const upperArmLength = vectorMagnitude(upperArmVec);
        const forearmLength = vectorMagnitude(forearmVec);
        const totalArmLength = upperArmLength + forearmLength;
        
        // Store baseline arm length (use a rolling average for stability)
        if (!this.armLengthHistory) {
            this.armLengthHistory = [];
        }
        
        // Add current length to history
        this.armLengthHistory.push(totalArmLength);
        if (this.armLengthHistory.length > 20) {
            this.armLengthHistory.shift(); // Keep only last 20 samples
        }
        
        // Calculate baseline as the maximum length seen (representing full extension sideways)
        const maxArmLength = Math.max(...this.armLengthHistory);
        const avgArmLength = this.armLengthHistory.reduce((a, b) => a + b) / this.armLengthHistory.length;
        
        // Use max length as baseline (sideways extension should be longest)
        const baselineLength = maxArmLength;
        
        // Calculate length ratio (current length / baseline length)
        const lengthRatio = totalArmLength / baselineLength;
        
        // More conservative threshold - only detect front if significantly shorter
        const FORESHORTENING_THRESHOLD = 0.75; // Arms must be 25% shorter to be considered "front"
        
        let shoulderX = 0; // Start with sideways (original position)
        let isHandsFront = false;
        
        // Check if arms are roughly horizontal
        const isHorizontal = Math.abs(upperArmAngle) < Math.PI/3; // Within 60 degrees of horizontal (more lenient)
        
        // Only consider "front" if significantly shortened AND horizontal
        if (isHorizontal && lengthRatio < FORESHORTENING_THRESHOLD && this.armLengthHistory.length > 10) {
            // Arms are horizontal and significantly shortened -> hands front
            isHandsFront = true;
            shoulderX = 1.5;
        } else if (upperArmAngle < -Math.PI/4) { // Upper quadrant (hands up)
            shoulderX = 1.5;
        } else if (upperArmAngle > Math.PI/4) { // Lower quadrant (hands down) 
            shoulderX = 1.5;
        } else { // Side range (hands sideways) - horizontal but full/normal length
            shoulderX = 0;
        }
        
        // For smoother transition, use interpolation for non-front positions
        if (!isHandsFront) {
            const absAngle = Math.abs(upperArmAngle);
            if (absAngle > Math.PI/4) {
                shoulderX = 1.5; // Both up and down use 1.5
            } else {
                // Smooth transition from sideways (0) to up/down (1.5)
                const t = absAngle / (Math.PI/4); // 0 to 1
                shoulderX = t * 1.5; // Interpolate 0 → 1.5
            }
        }
        
        shoulderX = smooth('l_shoulder_x', shoulderX);
        shoulderX = Math.max(-1.832, Math.min(1.919, shoulderX));
    
        // === SHOULDER Y (Front/Back based on your mapping) ===
        // Your mapping: Hands up = -1.5, Hands down = +1.5, Hands front = 0, Sideways = 0
        let shoulderY = 0;
        
        if (isHandsFront) {
            // Hands front position
            shoulderY = 0;
        } else if (upperArmAngle < -Math.PI/4) { // Hands up
            shoulderY = -1.5;
        } else if (upperArmAngle > Math.PI/4) { // Hands down
            shoulderY = 1.5;
        } else { // Hands sideways
            shoulderY = 0;
        }
        
        // Smooth interpolation for natural movement (only for non-front positions)
        if (!isHandsFront) {
            if (upperArmAngle < -Math.PI/4) {
                shoulderY = -1.5;
            } else if (upperArmAngle > Math.PI/4) {
                shoulderY = 1.5;
            } else {
                // Interpolate between up (-1.5) and down (+1.5) through sideways (0)
                const normalizedAngle = upperArmAngle / (Math.PI/4); // -1 to +1
                shoulderY = normalizedAngle * 1.5; // -1.5 to +1.5
            }
        }
        
        shoulderY = smooth('l_shoulder_y', shoulderY);
        shoulderY = Math.max(-2.094, Math.min(2.705, shoulderY));
    
        // === ARM Z (No rotation in 2D) ===
        let armZ = 0; // Keep at zero for pure 2D
        armZ = smooth('l_arm_z', armZ);
    
        // === ELBOW Y - SET TO 0 ===
        // Fixed elbow position at 0 (straight arm)
        let elbowY = 0;
        
        elbowY = smooth('l_elbow_y', elbowY);
        // No need to clamp since we're keeping it at 0
    
        // Update robot joints
        this.viewer?.updateJoint('l_shoulder_x', shoulderX);
        this.viewer?.updateJoint('l_shoulder_y', shoulderY);
        this.viewer?.updateJoint('l_arm_z', armZ);
        this.viewer?.updateJoint('l_elbow_y', elbowY);
    
        // 2D Debug info
        if (this.debugMode) {
            console.log('2D Mapping:', {
                upperArmAngle: (upperArmAngle * 180 / Math.PI).toFixed(1) + '°',
                forearmAngle: (forearmAngle * 180 / Math.PI).toFixed(1) + '°',
                totalArmLength: totalArmLength.toFixed(3),
                maxArmLength: (this.armLengthHistory ? Math.max(...this.armLengthHistory).toFixed(3) : 'N/A'),
                lengthRatio: lengthRatio.toFixed(3),
                samplesCount: this.armLengthHistory ? this.armLengthHistory.length : 0,
                isHandsFront: isHandsFront,
                shoulderX: shoulderX.toFixed(3),
                shoulderY: shoulderY.toFixed(3),
                elbowY: elbowY.toFixed(3)
            });
        }
    }
    
    // Additional helper: Detect specific 2D poses
    detect2DPose(poseLandmarks) {
        const shoulder = poseLandmarks[11];
        const elbow = poseLandmarks[13];
        const wrist = poseLandmarks[15];
        
        if (!shoulder || !elbow || !wrist) return "unknown";
        
        const upperArmVec = { x: elbow.x - shoulder.x, y: elbow.y - shoulder.y };
        const forearmVec = { x: wrist.x - elbow.x, y: wrist.y - elbow.y };
        const upperArmAngle = Math.atan2(upperArmVec.y, upperArmVec.x);
        
        // Calculate arm lengths for foreshortening detection
        const upperArmLength = Math.hypot(upperArmVec.x, upperArmVec.y);
        const forearmLength = Math.hypot(forearmVec.x, forearmVec.y);
        const totalArmLength = upperArmLength + forearmLength;
        
        // Use stored baseline or estimate (this is simplified - in real usage, baseline would be established)
        const estimatedBaseline = totalArmLength / 0.85; // Assume current might be shortened
        const lengthRatio = totalArmLength / estimatedBaseline;
        
        // Convert to degrees for easier understanding
        const angleDeg = upperArmAngle * 180 / Math.PI;
        
        // Check for hands front position (horizontal + shortened)
        const isHorizontal = Math.abs(angleDeg) < 45;
        const isShortened = lengthRatio < 0.85;
        
        if (isHorizontal && isShortened) return "hands_front";
        else if (angleDeg < -45) return "hands_up";
        else if (angleDeg > 45) return "hands_down";
        else if (Math.abs(angleDeg) < 15) return "hands_sideways";
        else return "transitioning";
    }
    
}