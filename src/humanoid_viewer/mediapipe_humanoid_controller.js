// mediapipe_pose_controller.js

import { PoseLandmarker, FilesetResolver } from "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision/vision_bundle.js";

export class MediaPipeHandController { // Consider renaming this to MediaPipePoseController for clarity
    constructor(viewerInstance, webcamElement) {
        this.viewer = viewerInstance;
        this.webcamElement = webcamElement; // The original webcam video element
        this.uploadedVideoElement = document.getElementById('uploadedVideo'); // New: Reference to uploaded video element
        this.videoInput = document.getElementById('videoInput'); // New: Reference to video file input
        this.playVideoButton = document.getElementById('playVideoButton'); // New: Reference to play video button
        this.useWebcamButton = document.getElementById('useWebcamButton'); // New: Reference to use webcam button
        this.videoWrapper = document.getElementById('videoWrapper'); // New: Reference to video input wrapper

        this.currentVideoSource = 'webcam'; // 'webcam' or 'uploaded'
        this.video = this.webcamElement; // This will point to the active video source

        this.poseLandmarker = null;
        this.runningMode = "VIDEO";
        this.webcamInitialized = false;
        this.lastVideoTime = -1;
        this.animationFrameId = null; // To store the ID of the requestAnimationFrame
        this.videoStream = null; // To store the webcam stream for stopping it

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
            'head_y': 0,
            'r_hip_y': 0, // Added for right leg
            'r_knee_y': 0, // Added for right leg
            'l_hip_y': 0, // Added for left leg
            'l_knee_y': 0  // Added for left leg
        };

        // History for arm length for foreshortening detection
        this.armLengthHistory_L = [];
        this.armLengthHistory_R = [];

        this.initMediaPipe();
        this.addEventListeners(); // New: Call to add event listeners
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
        this.setupWebcam(); // Start with webcam by default
    }

    addEventListeners() {
        // Handle URDF viewer events (if any, not directly related to mediapipe control)
        // For video upload
        this.videoInput.addEventListener('change', (event) => {
            const file = event.target.files[0];
            if (file) {
                this.uploadedVideoElement.src = URL.createObjectURL(file);
                this.playVideoButton.disabled = false;
                this.videoWrapper.classList.add('has-files');
                this.viewer.updateStatus(`Video '${file.name}' loaded. Click Play Video.`, 'success');
            } else {
                this.playVideoButton.disabled = true;
                this.videoWrapper.classList.remove('has-files');
                this.viewer.updateStatus('No video file selected.', 'error');
            }
        });

        this.playVideoButton.addEventListener('click', () => {
            if (this.uploadedVideoElement.src) {
                this.stopCurrentVideoSource();
                this.currentVideoSource = 'uploaded';
                this.video = this.uploadedVideoElement;
                this.webcamElement.style.display = 'none';
                this.uploadedVideoElement.style.display = 'block';
                this.uploadedVideoElement.play();
                this.viewer.updateStatus('Playing uploaded video...', 'loading');
                this.detectPosesInRealTime(); // Start detection for video
            }
        });

        this.uploadedVideoElement.addEventListener('play', () => {
            this.viewer.updateStatus('Video playing, mimicking motion.', 'success');
        });

        this.uploadedVideoElement.addEventListener('ended', () => {
            this.viewer.updateStatus('Video finished. Resetting robot.', 'success');
            this.resetRobotToInitialPose();
            this.drawLandmarks([]); // Clear overlay
            // Optionally, switch back to webcam or remain paused
            // this.setupWebcam(); 
        });

        this.uploadedVideoElement.addEventListener('pause', () => {
            this.viewer.updateStatus('Video paused.', 'loading');
        });

        this.uploadedVideoElement.addEventListener('error', (e) => {
            console.error('Error playing uploaded video:', e);
            this.viewer.updateStatus('Error playing video. Try another file.', 'error');
            this.resetRobotToInitialPose();
        });


        this.useWebcamButton.addEventListener('click', () => {
            this.stopCurrentVideoSource();
            this.currentVideoSource = 'webcam';
            this.video = this.webcamElement;
            this.uploadedVideoElement.style.display = 'none';
            this.webcamElement.style.display = 'block';
            this.setupWebcam(); // Re-initialize webcam
            this.viewer.updateStatus('Switching to webcam...', 'loading');
        });
    }

    stopCurrentVideoSource() {
        if (this.animationFrameId) {
            cancelAnimationFrame(this.animationFrameId);
            this.animationFrameId = null;
        }

        if (this.currentVideoSource === 'webcam' && this.videoStream) {
            this.videoStream.getTracks().forEach(track => track.stop());
            this.webcamElement.srcObject = null;
            this.webcamInitialized = false;
        } else if (this.currentVideoSource === 'uploaded' && !this.uploadedVideoElement.paused) {
            this.uploadedVideoElement.pause();
            this.uploadedVideoElement.currentTime = 0; // Reset video to start
        }
        this.resetRobotToInitialPose(); // Reset robot pose when switching/stopping
        this.drawLandmarks([]); // Clear overlay
    }


    setupWebcam() {
        if (this.webcamInitialized && this.video.srcObject) { // Already initialized and running
            console.log("Webcam already initialized.");
            this.viewer.updateStatus("Webcam active. Mimicking motion.", 'success');
            if (!this.animationFrameId) { // Ensure detection loop is running
                this.detectPosesInRealTime();
            }
            return;
        }

        if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
            navigator.mediaDevices.getUserMedia({ video: true })
                .then((stream) => {
                    this.videoStream = stream; // Store the stream
                    this.webcamElement.srcObject = stream;
                    this.webcamElement.addEventListener("canplay", () => {
                        this.webcamElement.play(); // Ensure webcam video plays
                        this.overlayCanvas = document.getElementById("overlay");
                        this.overlayCanvas.width = this.webcamElement.videoWidth;
                        this.overlayCanvas.height = this.webcamElement.videoHeight;
                        this.overlayCtx = this.overlayCanvas.getContext("2d");

                        this.webcamInitialized = true;
                        this.viewer.updateStatus("Webcam stream started. Starting pose detection.", 'success');
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
        if (this.video.paused || this.video.ended) {
            // Stop processing if video is paused or ended
            this.animationFrameId = null;
            return;
        }

        // Only detect if video (webcam or uploaded) is ready and has new frame
        if (this.videoInitialized && this.video.currentTime !== this.lastVideoTime) { // Check this.videoInitialized
            this.lastVideoTime = this.video.currentTime;
            
            // Check if the video element has valid dimensions before proceeding
            if (this.video.videoWidth === 0 || this.video.videoHeight === 0) {
                if (this.debugMode) console.log("Video element has zero dimensions, skipping detection.");
                this.animationFrameId = requestAnimationFrame(this.detectPosesInRealTime.bind(this));
                return;
            }

            const startTimeMs = performance.now();

            try {
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
            } catch (error) {
                console.error("Pose detection error:", error);
                // In case of error, revert to initial pose as a safe fallback
                this.resetRobotToInitialPose();
                this.drawLandmarks([]);
            }
        } else if (!this.videoInitialized && this.video.readyState >= 2) { // Check if video is ready to play
            this.overlayCanvas = document.getElementById("overlay");
            this.overlayCanvas.width = this.video.videoWidth;
            this.overlayCanvas.height = this.video.videoHeight;
            this.overlayCtx = this.overlayCanvas.getContext("2d");
            this.videoInitialized = true; // Mark as initialized
            if (this.debugMode) console.log("Video element dimensions updated for overlay.");
        } else {
            // No new video frame or video not ready, but we still want to keep the loop going
            if (this.debugMode) console.log("No new video frame or video not ready.");
            if (this.video.paused) {
                 this.drawLandmarks([]); // Clear overlay if paused
            }
        }
        this.animationFrameId = requestAnimationFrame(this.detectPosesInRealTime.bind(this));
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
        if (!this.overlayCtx || !this.overlayCanvas) {
            if (this.debugMode) console.warn("Overlay canvas not initialized for drawing.");
            return;
        }
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
            RIGHT_WRIST: 16,
            LEFT_HIP: 23,
            RIGHT_HIP: 24,
            LEFT_KNEE: 25,
            RIGHT_KNEE: 26,
            LEFT_ANKLE: 27,
            RIGHT_ANKLE: 28,
            LEFT_FOOT_INDEX: 31,
            RIGHT_FOOT_INDEX: 32
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

        // --- Right Leg Logic (Hip and Knee) ---
        if (!isValid(poseLandmarks[POSE.RIGHT_HIP]) ||
            !isValid(poseLandmarks[POSE.RIGHT_KNEE]) ||
            !isValid(poseLandmarks[POSE.RIGHT_ANKLE])) {
            if (this.debugMode) console.warn("Invalid right leg landmarks, defaulting to initial pose for right leg.");
            this.viewer?.updateJoint('r_hip_y', this.initialRobotPose.r_hip_y);
            this.viewer?.updateJoint('r_knee_y', this.initialRobotPose.r_knee_y);
        } else {
            const r_hip = poseLandmarks[POSE.RIGHT_HIP];
            const r_knee = poseLandmarks[POSE.RIGHT_KNEE];
            const r_ankle = poseLandmarks[POSE.RIGHT_ANKLE];

            if (this.debugMode) {
                console.groupCollapsed('Right Leg Debug (Raw Landmarks)');
                console.log(`R_Hip (24): x:${r_hip.x.toFixed(4)}, y:${r_hip.y.toFixed(4)}, z:${r_hip.z ? r_hip.z.toFixed(4) : 'N/A'}, vis:${r_hip.visibility ? r_hip.visibility.toFixed(4) : 'N/A'}`);
                console.log(`R_Knee (26): x:${r_knee.x.toFixed(4)}, y:${r_knee.y.toFixed(4)}, z:${r_knee.z ? r_knee.z.toFixed(4) : 'N/A'}, vis:${r_knee.visibility ? r_knee.visibility.toFixed(4) : 'N/A'}`);
                console.log(`R_Ankle (28): x:${r_ankle.x.toFixed(4)}, y:${r_ankle.y.toFixed(4)}, z:${r_ankle.z ? r_ankle.z.toFixed(4) : 'N/A'}, vis:${r_ankle.visibility ? r_ankle.visibility.toFixed(4) : 'N/A'}`);
                console.groupEnd();
            }

            // Hip_y: Controls thigh rotation (front/back)
            // Use the vertical position of the hip relative to a baseline (e.g., shoulder) or use thigh vector angle.
            // For now, let's use the difference in y between hip and knee, scaled.
            // This might need more sophisticated mapping if you want to control rotation of hip.
            let r_hip_y_val = 0; // Default or simple mapping

            // Femur vector (hip to knee)
            const femurVec_R = create2DVector(r_hip, r_knee);
            // Tibia vector (knee to ankle)
            const tibiaVec_R = create2DVector(r_knee, r_ankle);

            // r_knee_y: Controls the knee bend.
            // A straight leg is represented by r_knee_y = 0.
            // As the knee bends backward, r_knee_y should become more negative (down to -1.5 for "knee back").

            const currentKneeAngle_R = angleBetweenVectors(femurVec_R, tibiaVec_R); // Angle at human knee (radians)
            
            // For a straight leg, currentKneeAngle_R will be close to 0 radians (vectors point in same direction).
            // As the knee bends backward, currentKneeAngle_R will increase.
            // We want to map currentKneeAngle_R from 0 to some MAX_BEND_ANGLE_HUMAN to 0 to -1.5 for robot.

            // A typical maximum human knee bend is around 140-150 degrees (approx 2.4-2.6 radians).
            // Let's use 2.5 radians as a plausible max bend for scaling.
            const MAX_HUMAN_KNEE_BEND_RADIANS = 2.5; // Tune this value as needed

            // The mapping should be:
            // When currentKneeAngle_R = 0, r_knee_y_val = 0
            // When currentKneeAngle_R = MAX_HUMAN_KNEE_BEND_RADIANS, r_knee_y_val = -1.5
            let r_knee_y_val = -(currentKneeAngle_R / MAX_HUMAN_KNEE_BEND_RADIANS) * 1.5; 

            // Ensure the value does not go below the minimum limit (-1.5) and not above 0 (straight)
            r_knee_y_val = Math.max(-1.5, Math.min(0, r_knee_y_val));
            r_knee_y_val = smooth('r_knee_y', r_knee_y_val);
            this.viewer?.updateJoint('r_knee_y', r_knee_y_val);

            // Update robot joints for right leg
            this.viewer?.updateJoint('r_hip_y', r_hip_y_val); // Only setting to 0 for now, implement actual mapping for hip_y
            // r_knee_y is updated above

            if (this.debugMode) {
                console.log('Right Leg Results (Smoothed):', {
                    r_hip_y: r_hip_y_val.toFixed(3),
                    r_knee_y: r_knee_y_val.toFixed(3),
                    currentKneeAngle: (currentKneeAngle_R * 180 / Math.PI).toFixed(1) + '°'
                });
            }
        }

        // --- Left Leg Logic (Hip and Knee) ---
        if (!isValid(poseLandmarks[POSE.LEFT_HIP]) ||
            !isValid(poseLandmarks[POSE.LEFT_KNEE]) ||
            !isValid(poseLandmarks[POSE.LEFT_ANKLE])) {
            if (this.debugMode) console.warn("Invalid left leg landmarks, defaulting to initial pose for left leg.");
            this.viewer?.updateJoint('l_hip_y', this.initialRobotPose.l_hip_y);
            this.viewer?.updateJoint('l_knee_y', this.initialRobotPose.l_knee_y);
        } else {
            const l_hip = poseLandmarks[POSE.LEFT_HIP];
            const l_knee = poseLandmarks[POSE.LEFT_KNEE];
            const l_ankle = poseLandmarks[POSE.LEFT_ANKLE];

            if (this.debugMode) {
                console.groupCollapsed('Left Leg Debug (Raw Landmarks)');
                console.log(`L_Hip (23): x:${l_hip.x.toFixed(4)}, y:${l_hip.y.toFixed(4)}, z:${l_hip.z ? l_hip.z.toFixed(4) : 'N/A'}, vis:${l_hip.visibility ? l_hip.visibility.toFixed(4) : 'N/A'}`);
                console.log(`L_Knee (25): x:${l_knee.x.toFixed(4)}, y:${l_knee.y.toFixed(4)}, z:${l_knee.z ? l_knee.z.toFixed(4) : 'N/A'}, vis:${l_knee.visibility ? l_knee.visibility.toFixed(4) : 'N/A'}`);
                console.log(`L_Ankle (27): x:${l_ankle.x.toFixed(4)}, y:${l_ankle.y.toFixed(4)}, z:${l_ankle.z ? l_ankle.z.toFixed(4) : 'N/A'}, vis:${l_ankle.visibility ? l_ankle.visibility.toFixed(4) : 'N/A'}`);
                console.groupEnd();
            }

            let l_hip_y_val = 0; // Default or simple mapping

            const femurVec_L = create2DVector(l_hip, l_knee);
            const tibiaVec_L = create2DVector(l_knee, l_ankle);

            const currentKneeAngle_L = angleBetweenVectors(femurVec_L, tibiaVec_L);

            const MAX_HUMAN_KNEE_BEND_RADIANS = 2.5; // Consistent with right leg

            let l_knee_y_val = -(currentKneeAngle_L / MAX_HUMAN_KNEE_BEND_RADIANS) * 1.5;

            l_knee_y_val = Math.max(-1.5, Math.min(0, l_knee_y_val));
            l_knee_y_val = smooth('l_knee_y', l_knee_y_val);
            this.viewer?.updateJoint('l_knee_y', l_knee_y_val);

            this.viewer?.updateJoint('l_hip_y', l_hip_y_val);

            if (this.debugMode) {
                console.log('Left Leg Results (Smoothed):', {
                    l_hip_y: l_hip_y_val.toFixed(3),
                    l_knee_y: l_knee_y_val.toFixed(3),
                    currentKneeAngle: (currentKneeAngle_L * 180 / Math.PI).toFixed(1) + '°'
                });
            }
        }
    }
    
} // End of MediaPipeHandController class