using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace Robot
{
    /// <summary>
    /// Writes 24-joint body poses to a CSV file, BYTE-FORMAT IDENTICAL to
    /// ari-teleoperate-meta's PosePublisher body_poses_*.csv, so files produced here
    /// can be imported and replayed in ari's PoseDebugVisualizer.
    ///
    /// Layout (one row per frame):
    ///   timestamp, j0_px,j0_py,j0_pz,j0_qx,j0_qy,j0_qz,j0_qw, ... , j23_qw
    /// Numbers use ToString("R", InvariantCulture); timestamp uses Time.timeAsDouble
    /// (seconds), exactly as ari does.
    /// </summary>
    public class BodyPoseCsvLogger
    {
        private readonly TrackingData _trackingData = new TrackingData();

        private StreamWriter _writer;
        private string _path;
        private bool _recording;
        private int _rowCount;

        public bool IsRecording => _recording;
        // True once the CSV file is actually open and frames are being written
        // (recording can be armed but idle while waiting for tracker calibration).
        public bool IsWriting => _writer != null;
        public int RowCount => _rowCount;
        public string LastPath => _path;

        public void Start()
        {
            if (_recording) return;
            _recording = true;
            _rowCount = 0;
            Debug.Log("[BodyPoseCsvLogger] === RECORD ARMED === waiting for tracker data/calibration.");
        }

        public void Stop()
        {
            if (!_recording && _writer == null) return;
            _recording = false;
            Debug.Log($"[BodyPoseCsvLogger] === RECORD END === file: {_path} frames: {_rowCount}");
            Close();
        }

        public void Toggle()
        {
            if (_recording) Stop();
            else Start();
        }

        /// <summary>Call once per frame; no-ops unless recording and a valid frame is available.</summary>
        public void LogBodyPoses(double timestamp)
        {
            if (!_recording) return;
            if (!_trackingData.TryGetBodyJointPoses(out Vector3[] pos, out Quaternion[] rot)) return;

            int count = TrackingData.BodyJointCount;
            EnsureOpen(count);
            _rowCount++;

            var ci = CultureInfo.InvariantCulture;
            var sb = new StringBuilder(512);
            sb.Append(timestamp.ToString("R", ci));
            for (int i = 0; i < count; i++)
            {
                Vector3 p = pos[i];
                Quaternion q = rot[i];
                sb.Append(',').Append(p.x.ToString("R", ci));
                sb.Append(',').Append(p.y.ToString("R", ci));
                sb.Append(',').Append(p.z.ToString("R", ci));
                sb.Append(',').Append(q.x.ToString("R", ci));
                sb.Append(',').Append(q.y.ToString("R", ci));
                sb.Append(',').Append(q.z.ToString("R", ci));
                sb.Append(',').Append(q.w.ToString("R", ci));
            }
            _writer.WriteLine(sb.ToString());
        }

        private void EnsureOpen(int jointCount)
        {
            if (_writer != null) return;

            string fileName = $"body_poses_{DateTime.Now:yyyyMMdd_HHmmss}.csv";
            _path = Path.Combine("/sdcard/Download/", fileName);
            _writer = new StreamWriter(_path, false) { AutoFlush = true };

            var header = new StringBuilder("timestamp");
            for (int i = 0; i < jointCount; i++)
                header.Append($",j{i}_px,j{i}_py,j{i}_pz,j{i}_qx,j{i}_qy,j{i}_qz,j{i}_qw");
            _writer.WriteLine(header.ToString());

            Debug.Log($"[BodyPoseCsvLogger] === RECORD START === logging body poses to: {_path}");
        }

        public void Close()
        {
            if (_writer == null) return;
            _writer.Flush();
            _writer.Dispose();
            _writer = null;
        }
    }
}
