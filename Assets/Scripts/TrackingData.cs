using System;
using LitJson;
using Unity.XR.PICO.TOBSupport;
using Unity.XR.PXR;
using UnityEngine;
using UnityEngine.XR;
using CommonUsages = UnityEngine.XR.CommonUsages;
using InputDevice = UnityEngine.XR.InputDevice;

namespace Robot
{
    public class TrackingData
    {
        public static bool HeadOn { get; private set; }
        public static bool ControllerOn { get; private set; }
        public static bool HandTrackingOn { get; private set; }
        public static TrackingType TrackingTypeValue { get; private set; }

        private JsonData _motionTrackingJson = new JsonData();
        private JsonData _bodyTrackingJson = new JsonData();

        // BodyTrackerRole 0..23 == SMPL 24 joints.
        public const int BodyJointCount = 24;

        // Filled by TryGetBodyJointPoses and consumed by BOTH the JSON path
        // (GetBodyTracking) and BodyPoseCsvLogger, so body acquisition has one source.
        private readonly Vector3[] _bodyPositions = new Vector3[BodyJointCount];
        private readonly Quaternion[] _bodyRotations = new Quaternion[BodyJointCount];
        private BodyTrackingData _lastBodyData;
        private bool _lastBodyDataValid;
        private JsonData _controllerDataJson = new JsonData();
        private JsonData _leftControllerJson = new JsonData();
        private JsonData _rightControllerJson = new JsonData();

        private JsonData _stateData = new JsonData();
        private JsonData _handData = new JsonData();
        private JsonData _leftHandData = new JsonData();
        private JsonData _rightHandData = new JsonData();

        public static void SetHeadOn(bool on)
        {
            HeadOn = on;
        }

        public static void SetControllerOn(bool on)
        {
            ControllerOn = on;
        }

        public static void SetHandTrackingOn(bool on)
        {
            HandTrackingOn = on;
        }

        public static void SetTrackingType(TrackingType trackingType)
        {
            TrackingTypeValue = trackingType;
        }

        public static bool HasTracking
        {
            get
            {
                return (HeadOn || ControllerOn || HandTrackingOn ||
                        TrackingTypeValue > TrackingType.None);
            }
        }

        public void Get(ref JsonData totalData)
        {
            //sensor
            double predictTime = PXR_Enterprise.GetPredictedDisplayTime(); //毫秒
            predictTime = predictTime * 1000;
            totalData["predictTime"] = predictTime; //微秒，对应camera录制中帧插入的时间戳
            totalData["appState"] = _stateData;
            _stateData["focus"] = Application.isFocused;
            if (HeadOn)
            {
                //  if (jsonData == null) jsonData = new JsonData();
                //Right-handed coordinate system: X right, Y up, Z in
                PxrSensorState2 sensor = new PxrSensorState2();
                int sensorFrameIndex = 0;
                PXR_System.GetPredictedMainSensorStateNew(ref sensor, ref sensorFrameIndex);
                JsonData sensorJson = GetSensorJson(sensor);
                //     sensorJson["handMode"] = (int)HandModeValue;
                totalData["Head"] = sensorJson;
                // SendToServerMessage("GetHeadTracking", sensorJson.ToJson());
            }
            else
            {
                if (totalData.ContainsKey("Head"))
                    totalData.Remove("Head");
            }

            if (ControllerOn)
            {
                JsonData controller = GetLeftRightControllerJsonData(predictTime);
                totalData["Controller"] = controller;
            }
            else
            {
                if (totalData.ContainsKey("Controller"))
                    totalData.Remove("Controller");
            }

            if (HandTrackingOn)
            {
                //  ActiveInputDevice activeInputDevice = PXR_HandTracking.GetActiveInputDevice();
                //  if (activeInputDevice == ActiveInputDevice.HandTrackingActive)
                {
                    totalData["Hand"] = GetHandJsonData();
                }
            }
            else
            {
                if (totalData.ContainsKey("Hand"))
                    totalData.Remove("Hand");
            }

            if (TrackingTypeValue == TrackingType.Body)
            {
                // Acquire body joints via the single shared path (also used by the CSV logger).
                if (TryGetBodyJointPoses(out _, out _))
                {
                    totalData["Body"] = GetBodyTracking();
                }
            }
            else
            {
                if (totalData.ContainsKey("Body"))
                    totalData.Remove("Body");
            }

            if (TrackingTypeValue == TrackingType.Motion)
            {
                MotionTrackerMode trackingMode = PXR_MotionTracking.GetMotionTrackerMode();

                if (trackingMode == MotionTrackerMode.MotionTracking)
                {
                    JsonData json = GetMotionTracking();
                    totalData["Motion"] = json;
                }
            }
            else
            {
                if (totalData.ContainsKey("Motion"))
                    totalData.Remove("Motion");
            }


            long nsTime = Utils.GetCurrentTimestamp();
            totalData["timeStampNs"] = nsTime;
            ActiveInputDevice inputDevice = PXR_HandTracking.GetActiveInputDevice();
            totalData["Input"] = (int)inputDevice;
        }


        private JsonData GetMotionTracking()
        {
            MotionTrackerConnectState mtcs = new MotionTrackerConnectState();
            int ret = PXR_MotionTracking.GetMotionTrackerConnectStateWithSN(ref mtcs);

            int len = 0;
            JsonData joints;
            if (_motionTrackingJson.ContainsKey("joints"))
            {
                joints = _motionTrackingJson["joints"];
            }
            else
            {
                joints = new JsonData();
                joints.SetJsonType(JsonType.Array);
                _motionTrackingJson["joints"] = joints;
            }


            if (ret == 0)
            {
                if (mtcs.trackersSN.Length > 0)
                {
                    for (int i = 0; i < mtcs.trackerSum; i++)
                    {
                        string sn = mtcs.trackersSN[i].value.ToString().Trim();
                        if (!string.IsNullOrEmpty(sn))
                        {
                            //Obtain estimated position and rotation values for each somatosensory tracker
                            MotionTrackerLocations locations = new MotionTrackerLocations();
                            MotionTrackerConfidence confidence = new MotionTrackerConfidence();
                            int result =
                                PXR_MotionTracking.GetMotionTrackerLocations(mtcs.trackersSN[i], ref locations,
                                    ref confidence);

                            // If the position and rotation information are successfully obtained
                            if (result == 0)
                            {
                                JsonData joint;
                                if (len < joints.Count)
                                {
                                    joint = joints[len];
                                }
                                else
                                {
                                    joint = new JsonData();
                                    joints.Add(joint);
                                }

                                len++;
                                MotionTrackerLocation localLocation = locations.localLocation;

                                joint["p"] = GetPoseStr(localLocation.pose.Position, localLocation.pose.Orientation);
                                unsafe
                                {
                                    float* pVelo = localLocation.linearVelocity;
                                    float* pAcce = localLocation.linearAcceleration;
                                    float* pWVelo = localLocation.angularVelocity;
                                    float* pWAcce = localLocation.angularAcceleration;

                                    string va = pVelo[0] + "," + pVelo[1] + "," + pVelo[2] + "," + pWVelo[0] + "," +
                                                pWVelo[1] +
                                                "," + pWVelo[2];

                                    joint["va"] = va;
                                    string wva = pAcce[0] + "," + pAcce[1] + "," + pAcce[2] + "," + pWAcce[0] + "," +
                                                 pWAcce[1] +
                                                 "," + pWAcce[2];

                                    joint["wva"] = wva;
                                }

                                joint["sn"] = sn;
                            }
                        }
                    }
                }
            }

            _motionTrackingJson["len"] = len;
            return _motionTrackingJson;
        }


        /// <summary>
        /// Single source of truth for body-joint acquisition (old PICO SDK path):
        /// calib-state guard -> mode guard -> GetBodyTrackingData, with the same
        /// coordinate flip (PosZ / RotQz / RotQw negated) used by the JSON output.
        /// Fills shared buffers and caches the raw frame for velocity reuse.
        /// Used by both the JSON streaming path and BodyPoseCsvLogger.
        /// </summary>
        public bool TryGetBodyJointPoses(out Vector3[] positions, out Quaternion[] rotations)
        {
            positions = _bodyPositions;
            rotations = _bodyRotations;
            _lastBodyDataValid = false;

            int state = 0;
            PXR_Input.GetMotionTrackerCalibState(ref state);
            if (state != 1)
                return false;

            if (PXR_MotionTracking.GetMotionTrackerMode() != MotionTrackerMode.BodyTracking)
                return false;

            BodyTrackingGetDataInfo dataInfo = new BodyTrackingGetDataInfo();
            BodyTrackingData trackingData = new BodyTrackingData();
            PXR_MotionTracking.GetBodyTrackingData(ref dataInfo, ref trackingData);
            if (trackingData.roleDatas == null)
                return false;

            _lastBodyData = trackingData;
            _lastBodyDataValid = true;

            int count = Mathf.Min(trackingData.roleDatas.Length, BodyJointCount);
            for (int i = 0; i < count; i++)
            {
                var localPose = trackingData.roleDatas[i].localPose;
                _bodyPositions[i] = new Vector3(
                    (float)localPose.PosX, (float)localPose.PosY, -(float)localPose.PosZ);
                _bodyRotations[i] = new Quaternion(
                    (float)localPose.RotQx, (float)localPose.RotQy, -(float)localPose.RotQz, -(float)localPose.RotQw);
            }

            for (int i = count; i < BodyJointCount; i++)
            {
                _bodyPositions[i] = Vector3.zero;
                _bodyRotations[i] = Quaternion.identity;
            }

            return true;
        }

        private JsonData GetBodyTracking()
        {
            JsonData joints;
            if (_bodyTrackingJson.ContainsKey("joints"))
            {
                joints = _bodyTrackingJson["joints"];
            }
            else
            {
                joints = new JsonData();
                joints.SetJsonType(JsonType.Array);
                _bodyTrackingJson["joints"] = joints;
            }

            if (_lastBodyDataValid && _lastBodyData.roleDatas != null)
            {
                int len = Mathf.Min(_lastBodyData.roleDatas.Length, BodyJointCount);
                _motionTrackingJson["len"] = len;

                for (int i = 0; i < len; i++)
                {
                    JsonData joint;
                    if (i < joints.Count)
                    {
                        joint = joints[i];
                    }
                    else
                    {
                        joint = new JsonData();
                        joints.Add(joint);
                    }

                    joint["p"] = GetPoseStr(_bodyPositions[i], _bodyRotations[i]);

                    joint["t"] = _lastBodyData.roleDatas[i].localPose.TimeStamp;
                    unsafe
                    {
                        fixed (double* pVelo = _lastBodyData.roleDatas[i].velo, pAcce =
                                   _lastBodyData.roleDatas[i].acce, pWVelo =
                                   _lastBodyData.roleDatas[i].wvelo, pWAcce = _lastBodyData.roleDatas[i].wacce)
                        {
                            string va = pVelo[0] + "," + pVelo[1] + "," + pVelo[2] + "," + pAcce[0] + "," + pAcce[1] +
                                        "," + pAcce[2];

                            joint["va"] = va;
                            string wva = pWVelo[0] + "," + pWVelo[1] + "," + pWVelo[2] + "," + pWAcce[0] + "," +
                                         pWAcce[1] +
                                         "," + pWAcce[2];

                            joint["wva"] = wva;
                        }
                    }
                }
            }

            return _bodyTrackingJson;
        }


        private JsonData GetLeftRightControllerJsonData(double predictTime)
        {
            Vector3 leftPosition =
                PXR_Input.GetControllerPredictPosition(PXR_Input.Controller.LeftController, predictTime);
            Quaternion leftRotation =
                PXR_Input.GetControllerPredictRotation(PXR_Input.Controller.LeftController, predictTime);


            InputDevice left = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
            GetControllerJsonData(left, ref _leftControllerJson);
            _controllerDataJson["left"] = _leftControllerJson;
            _controllerDataJson["left"]["pose"] = GetPoseStr(leftPosition, leftRotation);

            Vector3 rightPosition =
                PXR_Input.GetControllerPredictPosition(PXR_Input.Controller.RightController, predictTime);
            Quaternion rightRotation =
                PXR_Input.GetControllerPredictRotation(PXR_Input.Controller.RightController, predictTime);

            InputDevice right = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            GetControllerJsonData(right, ref _rightControllerJson);
            _controllerDataJson["right"] = _rightControllerJson;
            _controllerDataJson["right"]["pose"] = GetPoseStr(rightPosition, rightRotation);

            return _controllerDataJson;
        }

        private static void GetControllerJsonData(InputDevice controllerDevice, ref JsonData json)
        {
            controllerDevice.TryGetFeatureValue(CommonUsages.primary2DAxis, out var axis2D);
            controllerDevice.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out var axisClick);
            controllerDevice.TryGetFeatureValue(CommonUsages.grip, out var grip);
            controllerDevice.TryGetFeatureValue(CommonUsages.trigger, out var trigger);
            controllerDevice.TryGetFeatureValue(CommonUsages.primaryButton, out var primaryButton);
            controllerDevice.TryGetFeatureValue(CommonUsages.secondaryButton, out var secondaryButton);
            controllerDevice.TryGetFeatureValue(CommonUsages.menuButton, out var menuButton);

            json["axisX"] = axis2D.x;
            json["axisY"] = axis2D.y;
            json["axisClick"] = axisClick;
            json["grip"] = grip;
            json["trigger"] = trigger;
            json["primaryButton"] = primaryButton;
            json["secondaryButton"] = secondaryButton;
            json["menuButton"] = menuButton;
        }


        private JsonData GetHandJsonData()
        {
            HandJointLocations leftHand = new HandJointLocations();
            if (PXR_HandTracking.GetJointLocations(HandType.HandLeft, ref leftHand))
            {
                GetHandTrackingData(leftHand, ref _leftHandData);
                _handData["leftHand"] = _leftHandData;
            }

            HandJointLocations rightHand = new HandJointLocations();

            if (PXR_HandTracking.GetJointLocations(HandType.HandRight, ref rightHand))
            {
                GetHandTrackingData(rightHand, ref _rightHandData);
                _handData["rightHand"] = _rightHandData;
            }

            return _handData;
        }

        private JsonData GetSensorJson(PxrSensorState2 sensorState)
        {
            JsonData jsonData = new JsonData();
            PxrPosef pose = sensorState.pose;

            Vector3 pos = new Vector3(pose.position.x, pose.position.y, pose.position.z);
            Quaternion rot = new Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w);
            jsonData["pose"] = GetPoseStr(pos, rot);
            jsonData["status"] = sensorState.status;

            //  jsonData["timeStampNs"] = sensorState.poseTimeStampNs;
            return jsonData;
        }


        private void GetHandTrackingData(HandJointLocations handJoint, ref JsonData json)
        {
            json["isActive"] = handJoint.isActive;
            json["count"] = handJoint.jointCount;
            json["scale"] = handJoint.handScale;

            JsonData jointLocationsJson;
            if (json.ContainsKey("HandJointLocations"))
            {
                jointLocationsJson = json["HandJointLocations"];
            }
            else
            {
                jointLocationsJson = new JsonData();
                jointLocationsJson.SetJsonType(JsonType.Array);
                json["HandJointLocations"] = jointLocationsJson;
            }

            for (int i = 0; i < handJoint.jointCount; i++)
            {
                JsonData jointJson;
                if (i < jointLocationsJson.Count)
                {
                    jointJson = jointLocationsJson[i];
                }
                else
                {
                    jointJson = new JsonData();
                    jointLocationsJson.Add(jointJson);
                }

                Posef posef = handJoint.jointLocations[i].pose;
                jointJson["p"] = GetPoseStr(posef.Position, posef.Orientation);
                jointJson["s"] = ((ulong)handJoint.jointLocations[i].locationStatus);
                jointJson["r"] = handJoint.jointLocations[i].radius;
            }
        }

        private string GetPoseStr(Vector3 position, Quaternion rotation)
        {
            return position.x.ToString("R") + "," + position.y.ToString("R") + "," + position.z.ToString("R") + "," +
                   rotation.x.ToString("R") + "," + rotation.y.ToString("R") + "," + rotation.z.ToString("R") + "," +
                   rotation.w.ToString("R");
        }

        private string GetPoseStr(Vector3f position, Quatf rotation)
        {
            return position.x.ToString("R") + "," + position.y.ToString("R") + "," + position.z.ToString("R") + "," +
                   rotation.x.ToString("R") + "," + rotation.y.ToString("R") + "," + rotation.z.ToString("R") + "," +
                   rotation.w.ToString("R");
        }

        public enum HandMode
        {
            Non = 0,
            Controller = 1,
            Hand = 2
        }


        public enum TrackingType
        {
            None = 0,
            Body = 1,
            Motion = 2
        }
    }
}
