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
                const result = await this.poseLandmarker.detectForVideo(this.video, startTimeMs);

                if (result.landmarks && result.landmarks.length > 0) {
                    const poseLandmarks = result.landmarks[0];
                    console.log("Pose Landmarks:", poseLandmarks);
                    this.mapLandmarksToRobot(poseLandmarks);
                    this.drawLandmarks(poseLandmarks);
                } else {
                    this.drawLandmarks([]); // Clear overlay if no pose
                }
            } catch (error) {
                console.error("Pose detection error:", error);
            }
        }
        requestAnimationFrame(this.detectPosesInRealTime.bind(this));
    }

    mapLandmarksToRobot(poseLandmarks) {
        // Very simple example mapping (extend based on your robot joints)
        // You can modify this mapping as per your robot structure

        const jointMapping = {
            'left_shoulder': 'joint_left_shoulder',
            'right_shoulder': 'joint_right_shoulder',
            'left_elbow': 'joint_left_elbow',
            'right_elbow': 'joint_right_elbow'
        };

        // Just as an example, calculate very simple horizontal angles between shoulder and elbow
        const calculateAngle = (p1, p2) => {
            const dx = p2.x - p1.x;
            const dy = p2.y - p1.y;
            return Math.atan2(dy, dx);
        };

        const keypoints = {
            'left_shoulder': poseLandmarks[11],
            'right_shoulder': poseLandmarks[12],
            'left_elbow': poseLandmarks[13],
            'right_elbow': poseLandmarks[14]
        };

        for (const [landmarkName, jointName] of Object.entries(jointMapping)) {
            const parts = landmarkName.split('_');
            const side = parts[0];
            const joint = parts[1];

            if (joint === 'shoulder') {
                // For shoulder joint, no relative movement for now
                const angle = 0;
                if (this.viewer && this.viewer.updateJoint) {
                    this.viewer.updateJoint(jointName, angle);
                }
            } else if (joint === 'elbow') {
                const shoulderKey = `${side}_shoulder`;
                const elbowKey = `${side}_elbow`;

                const shoulder = keypoints[shoulderKey];
                const elbow = keypoints[elbowKey];

                if (shoulder && elbow) {
                    const angle = calculateAngle(shoulder, elbow);
                    if (this.viewer && this.viewer.updateJoint) {
                        this.viewer.updateJoint(jointName, angle);
                    }
                }
            }
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

        // Define connections for the pose skeleton
        // These connections cover the full MediaPipe PoseLandmarker_heavy model
        const connections = [
            // Torso
            [11, 12], // Shoulders
            [23, 24], // Hips
            [11, 23], [12, 24], // Left and Right torso connections

            // Left Arm
            [11, 13], // Left Shoulder to Elbow
            [13, 15], // Left Elbow to Wrist
            [15, 17], [17, 19], [19, 15], // Left Hand (Thumb, Index, Pinky to Wrist)
            [15, 21], // Left Wrist to Finger 1
            [15, 17], [17,19], [19,21], // Left Hand (Fingers)
            [15, 17], // Left Wrist to Thumb CMC
            [17, 19], // Left Thumb CMC to MP
            [19, 21], // Left Thumb MP to IP
            [15, 17], [17, 19], [19, 21], // Left Hand (fingers, generalized)
            
            // Left Hand - specific connections from documentation (adjust based on actual landmark usage for hand)
            // [15,17], [17,19], [19,21], // Index (wrist to thumb, etc.) These might need to be refined if hand landmarks are used.
            // For general pose, these are typically covered by wrist and general finger tips.
            
            // Right Arm
            [12, 14], // Right Shoulder to Elbow
            [14, 16], // Right Elbow to Wrist
            [16, 18], [18, 20], [20, 16], // Right Hand (Thumb, Index, Pinky to Wrist)
            [16, 22], // Right Wrist to Finger 1
            [16, 18], [18,20], [20,22], // Right Hand (Fingers)
            
            // Left Leg
            [23, 25], // Left Hip to Knee
            [25, 27], // Left Knee to Ankle
            [27, 29], [29, 31], [31, 27], // Left Foot (Ankle to Heel, Heel to Toes, Toes to Ankle)
            
            // Right Leg
            [24, 26], // Right Hip to Knee
            [26, 28], // Right Knee to Ankle
            [28, 30], [30, 32], [32, 28], // Right Foot (Ankle to Heel, Heel to Toes, Toes to Ankle)

            // Face (if included in the heavy model and you wish to draw them)
            // This assumes landmarks 0-10 are for the face. Check MediaPipe documentation.
            // For simplicity, connecting nose to eyes and eyes to ears.
            [0, 1], [0, 2], // Nose to eyes
            [1, 3], [2, 4], // Eyes to ears
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
}