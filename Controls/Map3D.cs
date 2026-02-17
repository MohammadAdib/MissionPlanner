using GMap.NET;
using GMap.NET.Internals;
using GMap.NET.MapProviders;
using GMap.NET.WindowsForms;
using Microsoft.Scripting.Utils;
using MissionPlanner.Utilities;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.ES20;
using OpenTK.Input;
using OpenTK.Platform;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;
using MathHelper = MissionPlanner.Utilities.MathHelper;
using MouseEventArgs = System.Windows.Forms.MouseEventArgs;
using Timer = System.Windows.Forms.Timer;
using Vector3 = OpenTK.Vector3;

namespace MissionPlanner.Controls
{
    public partial class Map3D : GLControl, IDeactivate
    {
        public static Map3D instance;
        private static GraphicsMode _graphicsMode;

        #region Constants
        private const double HEADING_LINE_LENGTH = 100; // meters
        private const double TURN_RADIUS_ARC_LENGTH = 200; // meters
        private const int TURN_RADIUS_SEGMENTS = 50;
        private const double ADSB_MAX_DISTANCE = 50000; // 50km
        private const double ADSB_RED_DISTANCE = 5000; // 5km
        private const double ADSB_YELLOW_DISTANCE = 10000; // 10km
        private const double ADSB_GREEN_DISTANCE = 20000; // 20km
        private const int ADSB_CIRCLE_SEGMENTS = 24;
        private const double WAYPOINT_MIN_DISTANCE = 61.0; // 200 feet in meters
        #endregion

        private static GraphicsMode GetGraphicsMode()
        {
            if (_graphicsMode != null)
                return _graphicsMode;

            // Prefer a 32-bit color buffer, 24-bit depth, 8-bit stencil, and 4x MSAA.
            try
            {
                _graphicsMode = new GraphicsMode(new ColorFormat(32), 24, 8, 4);
                return _graphicsMode;
            }
            catch
            {
                // Fall back to no multisampling if the platform/driver rejects MSAA.
                try
                {
                    _graphicsMode = new GraphicsMode(new ColorFormat(32), 24, 8, 0);
                    return _graphicsMode;
                }
                catch
                {
                    _graphicsMode = GraphicsMode.Default;
                    return _graphicsMode;
                }
            }
        }

        #region Helper Methods
        /// <summary>
        /// Gets a color based on distance for ADSB aircraft visualization.
        /// Red for close aircraft, yellow for medium distance, green for far.
        /// </summary>
        /// <param name="distance">Distance to aircraft in meters</param>
        /// <param name="isGrounded">Whether the aircraft is on the ground</param>
        /// <returns>RGBA color values (0.0-1.0)</returns>
        private (float r, float g, float b, float a) GetADSBDistanceColor(double distance, bool isGrounded)
        {
            if (isGrounded)
            {
                return (0.7f, 0.7f, 0.7f, 1.0f); // Light gray for grounded
            }

            if (distance <= ADSB_RED_DISTANCE)
            {
                return (1.0f, 0.0f, 0.0f, 1.0f); // Red
            }
            else if (distance <= ADSB_YELLOW_DISTANCE)
            {
                // Interpolate red to yellow
                float t = (float)((distance - ADSB_RED_DISTANCE) / (ADSB_YELLOW_DISTANCE - ADSB_RED_DISTANCE));
                return (1.0f, t, 0.0f, 1.0f);
            }
            else if (distance <= ADSB_GREEN_DISTANCE)
            {
                // Interpolate yellow to green
                float t = (float)((distance - ADSB_YELLOW_DISTANCE) / (ADSB_GREEN_DISTANCE - ADSB_YELLOW_DISTANCE));
                return (1.0f - t, 1.0f, 0.0f, 1.0f);
            }
            else
            {
                return (0.0f, 1.0f, 0.0f, 1.0f); // Green
            }
        }

        /// <summary>
        /// Calculates billboard orientation vectors for a point facing the camera.
        /// </summary>
        /// <param name="posX">Position X</param>
        /// <param name="posY">Position Y</param>
        /// <param name="posZ">Position Z</param>
        /// <param name="camX">Camera X</param>
        /// <param name="camY">Camera Y</param>
        /// <param name="camZ">Camera Z</param>
        /// <returns>Right and Up vectors for billboard orientation, or null if too close to camera</returns>
        private (double rightX, double rightY, double rightZ, double upX, double upY, double upZ)?
            CalculateBillboardOrientation(double posX, double posY, double posZ, double camX, double camY, double camZ)
        {
            // Calculate direction to camera
            double dx = posX - camX;
            double dy = posY - camY;
            double dz = posZ - camZ;
            double distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < 1.0)
                return null; // Too close to camera

            // Normalize view direction
            double viewDirX = dx / distance;
            double viewDirY = dy / distance;
            double viewDirZ = dz / distance;

            // Right vector (cross product of view dir with world up [0,0,1])
            double rightX = viewDirY;
            double rightY = -viewDirX;
            double rightZ = 0;
            double rightLen = Math.Sqrt(rightX * rightX + rightY * rightY);

            if (rightLen > 0.001)
            {
                rightX /= rightLen;
                rightY /= rightLen;
            }
            else
            {
                // Looking straight up/down, use arbitrary right
                rightX = 1;
                rightY = 0;
            }

            // Up vector (cross product of right with view dir)
            double upX = rightY * viewDirZ - rightZ * viewDirY;
            double upY = rightZ * viewDirX - rightX * viewDirZ;
            double upZ = rightX * viewDirY - rightY * viewDirX;

            return (rightX, rightY, rightZ, upX, upY, upZ);
        }

        /// <summary>
        /// Generates vertices for a billboarded circle facing the camera.
        /// </summary>
        /// <param name="centerX">Center X position</param>
        /// <param name="centerY">Center Y position</param>
        /// <param name="centerZ">Center Z position</param>
        /// <param name="radius">Circle radius</param>
        /// <param name="segments">Number of segments</param>
        /// <param name="r">Red color component (0-1)</param>
        /// <param name="g">Green color component (0-1)</param>
        /// <param name="b">Blue color component (0-1)</param>
        /// <param name="a">Alpha component (0-1)</param>
        /// <param name="vertices">List to add vertices to</param>
        private void AddBillboardCircleVertices(
            double centerX, double centerY, double centerZ,
            double radius, int segments,
            float r, float g, float b, float a,
            double rightX, double rightY, double rightZ,
            double upX, double upY, double upZ,
            List<float> vertices)
        {
            for (int i = 0; i < segments; i++)
            {
                double angle1 = (2 * Math.PI * i) / segments;
                double angle2 = (2 * Math.PI * (i + 1)) / segments;

                double cos1 = Math.Cos(angle1);
                double sin1 = Math.Sin(angle1);
                double cos2 = Math.Cos(angle2);
                double sin2 = Math.Sin(angle2);

                // Point 1: center + radius * (cos * right + sin * up)
                double x1 = centerX + radius * (cos1 * rightX + sin1 * upX);
                double y1 = centerY + radius * (cos1 * rightY + sin1 * upY);
                double z1 = centerZ + radius * (cos1 * rightZ + sin1 * upZ);

                // Point 2
                double x2 = centerX + radius * (cos2 * rightX + sin2 * upX);
                double y2 = centerY + radius * (cos2 * rightY + sin2 * upY);
                double z2 = centerZ + radius * (cos2 * rightZ + sin2 * upZ);

                // Add as separate line segment (x, y, z, r, g, b, a for each vertex)
                vertices.AddRange(new float[] { (float)x1, (float)y1, (float)z1, r, g, b, a });
                vertices.AddRange(new float[] { (float)x2, (float)y2, (float)z2, r, g, b, a });
            }
        }
        #endregion

        int green = 0;
        int greenAlt = 0;

        // Plane STL model
        private STLModelLoader _stlLoader = new STLModelLoader();
        private int _planeVBO = 0;
        private int _planeNormalVBO = 0;
        private bool _settingsLoaded = false;
        // Plane position and rotation for current frame
        private double _planeDrawX, _planeDrawY, _planeDrawZ;
        private float _planeRoll, _planePitch, _planeYaw;
        // Disconnected mode camera look direction (yaw/pitch for free-look)
        private double _disconnectedCameraYaw = 0.0;   // Horizontal look angle in degrees
        private double _disconnectedCameraPitch = 0.0; // Vertical look angle in degrees (positive = up)
        // Debounce for 2D map position updates when disconnected
        private PointLatLngAlt _lastMap2DPosition = PointLatLngAlt.Zero;
        private DateTime _lastMap2DPositionChangeTime = DateTime.MinValue;
        // Configurable camera and plane settings
        private double _cameraDist = 0.8;    // Distance from plane
        private double _cameraAngle = 0.0;   // Angle offset from behind plane (degrees, 0=behind, 90=right, -90=left)
        private double _cameraHeight = 0.2;  // Height above plane
        private float _planeScaleMultiplier = 1.0f; // 1.0 = 1 meter wingspan
        private float _cameraFOV = 60f; // Field of view in degrees
        private Color _planeColor = Color.Red;
        private int _whitePlaneTexture = 0; // White texture for plane rendering
        // Heading indicator line options
        private bool _showHeadingLine = true;
        private bool _showNavBearingLine = true;
        private bool _showGpsHeadingLine = true;
        private bool _showTurnRadius = true;
        private bool _showTrail = false;
        private bool _fpvMode = false; // First-person view mode - camera at aircraft position
        private bool _diskCacheTiles = true; // Cache tiles to disk for faster loading
        private double _waypointMarkerSize = 60; // Half-size of waypoint markers in meters
        private double _adsbCircleSize = 500; // Diameter of ADSB aircraft circles in meters
        // Trail (flight path history) - stored as absolute UTM coordinates (X, Y, Z)
        private List<double[]> _trailPoints = new List<double[]>();
        private int _trailUtmZone = -999;
        private Lines _trailLine = null;
        private int _trailStableFrames = 0; // Count frames with stable telemetry before recording
        private const int TrailStabilityThreshold = 30; // Frames to wait for stable altitude data
        // ADSB aircraft hit testing - stores screen positions and data for tooltip
        private List<ADSBScreenPosition> _adsbScreenPositions = new List<ADSBScreenPosition>();
        private ToolTip _adsbToolTip;
        private adsb.PointLatLngAltHdg _lastHoveredADSB = null;
        double cameraX, cameraY, cameraZ; // camera coordinates

        double lookX, lookY, lookZ; // camera look-at coordinates

        // image zoom level - automatic LOD based on altitude/distance
        private const int zoom = 17;
        private const int zoomLevelOffset = 4;
        private const int minZoomFloor = 13;

        /// <summary>
        /// Zoom levels to drop at altitude. 1 level per 500m.
        /// Capped so effectiveZoom never drops below minZoomFloor.
        /// </summary>
        private int AltitudeZoomDrop => Math.Min(zoom - minZoomFloor, (int)Math.Floor(Math.Max(0, _center.Alt) / 500.0));

        private int effectiveZoom => zoom - AltitudeZoomDrop;

        private int minzoom => Math.Max(minZoomFloor, effectiveZoom - zoomLevelOffset);
        private MyButton btn_configure;
        private SemaphoreSlim textureSemaphore = new SemaphoreSlim(1, 1);
        private Timer timer1;
        private bool _stopRequested;
        private System.ComponentModel.IContainer components;
        private PointLatLngAlt _center { get; set; } = new PointLatLngAlt(0, 0, 100);

        /// <summary>
        /// Returns true if the vehicle is connected and reporting a valid location (not 0,0)
        /// </summary>
        private bool IsVehicleConnected
        {
            get
            {
                if (MainV2.comPort?.BaseStream?.IsOpen != true)
                    return false;
                var loc = MainV2.comPort?.MAV?.cs?.Location;
                if (loc == null || (loc.Lat == 0 && loc.Lng == 0))
                    return false;
                return true;
            }
        }

        public PointLatLngAlt LocationCenter
        {
            get { return _center; }
            set
            {
                bool positionChanged = _center.Lat != value.Lat || _center.Lng != value.Lng;

                _centerTime = DateTime.Now;
                _center.Lat = value.Lat;
                _center.Lng = value.Lng;
                _center.Alt = value.Alt;

                // Initialize or update UTM zone if needed
                if (utmzone == -999 || utmzone != value.GetUTMZone() || llacenter.GetDistance(_center) > 10000)
                {
                    utmzone = value.GetUTMZone();
                    // set our pos
                    llacenter = value;
                    utmcenter = new double[] {0, 0};
                    // update a virtual center based on llacenter
                    utmcenter = convertCoords(value);
                    textureid.ForEach(a => a.Value.Cleanup());
                    textureid.Clear();
                    _forceRefreshTiles = true;
                }

                if (positionChanged)
                    this.Invalidate();
            }
        }

        private MissionPlanner.Utilities.Vector3 _velocity = new MissionPlanner.Utilities.Vector3();

        MissionPlanner.Utilities.Vector3 _rpy = new MissionPlanner.Utilities.Vector3();

        public MissionPlanner.Utilities.Vector3 rpy
        {
            get { return _rpy; }
            set
            {
                _rpy.X = (float) Math.Round(value.X, 2);
                _rpy.Y = (float) Math.Round(value.Y, 2);
                _rpy.Z = (float) Math.Round(value.Z, 2);
                this.Invalidate();
            }
        }

        public List<Locationwp> WPs { get; set; }

        public Map3D() : base(GetGraphicsMode())
        {
            instance = this;

            // Load settings early, before any rendering
            _cameraDist = Settings.Instance.GetDouble("map3d_camera_dist", 0.8);
            _cameraHeight = Settings.Instance.GetDouble("map3d_camera_height", 0.2);
            _cameraAngle = Settings.Instance.GetDouble("map3d_camera_angle", 0.0);
            _planeScaleMultiplier = (float)Settings.Instance.GetDouble("map3d_mav_scale", Settings.Instance.GetDouble("map3d_plane_scale", 1.0));
            _cameraFOV = (float)Settings.Instance.GetDouble("map3d_fov", 60);
            _stlLoader.CustomSTLPath = Settings.Instance.GetString("map3d_plane_stl_path", "");
            try
            {
                int colorArgb = Settings.Instance.GetInt32("map3d_mav_color", Settings.Instance.GetInt32("map3d_plane_color", Color.Red.ToArgb()));
                _planeColor = Color.FromArgb(colorArgb);
            }
            catch { _planeColor = Color.Red; }
            _showHeadingLine = Settings.Instance.GetBoolean("map3d_show_heading", true);
            _showNavBearingLine = Settings.Instance.GetBoolean("map3d_show_nav_bearing", true);
            _showGpsHeadingLine = Settings.Instance.GetBoolean("map3d_show_gps_heading", true);
            _showTurnRadius = Settings.Instance.GetBoolean("map3d_show_turn_radius", true);
            _showTrail = Settings.Instance.GetBoolean("map3d_show_trail", false);
            _fpvMode = Settings.Instance.GetBoolean("map3d_fpv_mode", false);
            _diskCacheTiles = Settings.Instance.GetBoolean("map3d_disk_cache_tiles", true);
            _waypointMarkerSize = Settings.Instance.GetDouble("map3d_waypoint_marker_size", 60);
            _adsbCircleSize = Settings.Instance.GetDouble("map3d_adsb_size", 500);

            InitializeComponent();
            Click += OnClick;
            MouseMove += OnMouseMove;
            MouseDown += OnMouseDown;
            MouseUp += OnMouseUp;
            MouseWheel += OnMouseWheel;
            MouseDoubleClick += OnMouseDoubleClick;

            // Initialize ADSB tooltip
            _adsbToolTip = new ToolTip();
            _adsbToolTip.AutoPopDelay = 10000;
            _adsbToolTip.InitialDelay = 0;
            _adsbToolTip.ReshowDelay = 0;
            _adsbToolTip.ShowAlways = true;
            core.OnMapOpen();
            type = GMap.NET.MapProviders.GoogleSatelliteMapProvider.Instance;
            prj = type.Projection;
            LocationCenter = LocationCenter.newpos(0, 0.1);
            // Disable VSync for smoother rendering
            try
            {
                VSync = false;
            }
            catch { }
            this.Invalidate();
        }

        private void OnMouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                // Start dragging to adjust camera Y (side) and Z (height)
                _isDragging = true;
                _dragStartX = e.X;
                _dragStartY = e.Y;
            }
        }

        private void OnMouseUp(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left && _isDragging)
            {
                _isDragging = false;
                // Don't save - drag changes are temporary
            }
        }

        private void OnMouseMove(object sender, MouseEventArgs e)
        {
            mousex = e.X;
            mousey = e.Y;

            // Handle left-drag behavior based on connection state
            if (_isDragging)
            {
                int deltaX = mousex - _dragStartX;
                int deltaY = mousey - _dragStartY;

                if (IsVehicleConnected)
                {
                    // Connected: rotate camera around vehicle (X) and adjust height (Y)
                    _cameraAngle += deltaX * 0.5; // Degrees per pixel
                    _cameraHeight = Math.Max(-5.0, Math.Min(5.0, _cameraHeight + deltaY * 0.005));
                }
                else
                {
                    // Disconnected: free-look - rotate camera view direction (reversed for natural feel)
                    _disconnectedCameraYaw -= deltaX * 0.3; // Horizontal rotation (reversed)
                    _disconnectedCameraPitch = Math.Max(-89.0, Math.Min(89.0, _disconnectedCameraPitch + deltaY * 0.3)); // Vertical rotation (reversed, clamped to avoid gimbal lock)
                }

                _dragStartX = mousex;
                _dragStartY = mousey;
                return;
            }

            try
            {
                mousePosition = getMousePos(mousex, mousey);
            } catch { }

            // Check for ADSB aircraft hover
            CheckADSBHover(e.X, e.Y);
        }

        /// <summary>
        /// Checks if the mouse is hovering over an ADSB aircraft circle and shows tooltip.
        /// </summary>
        private void CheckADSBHover(int mouseX, int mouseY)
        {
            adsb.PointLatLngAltHdg hoveredPlane = null;
            double hoveredDistance = 0;

            foreach (var pos in _adsbScreenPositions)
            {
                float dx = mouseX - pos.ScreenX;
                float dy = mouseY - pos.ScreenY;
                float dist = (float)Math.Sqrt(dx * dx + dy * dy);

                if (dist <= pos.Radius)
                {
                    hoveredPlane = pos.PlaneData;
                    hoveredDistance = pos.DistanceToOwn;
                    break;
                }
            }

            if (hoveredPlane != null)
            {
                if (!object.ReferenceEquals(_lastHoveredADSB, hoveredPlane))
                {
                    _lastHoveredADSB = hoveredPlane;

                    // Build tooltip text similar to 2D map
                    var sb = new System.Text.StringBuilder();
                    sb.AppendLine("ICAO: " + hoveredPlane.Tag);
                    sb.AppendLine("Callsign: " + hoveredPlane.CallSign);

                    // Type/Category
                    string typeCategory = "";
                    try
                    {
                        var raw = hoveredPlane.Raw as Newtonsoft.Json.Linq.JObject;
                        if (raw != null && raw.ContainsKey("t"))
                        {
                            typeCategory = raw["t"].ToString() + " ";
                        }
                    }
                    catch { }
                    typeCategory += hoveredPlane.GetCategoryFriendlyString();
                    sb.AppendLine("Type\\Category: " + typeCategory);

                    sb.AppendLine("Squawk: " + hoveredPlane.Squawk.ToString());
                    sb.AppendLine("Altitude: " + (hoveredPlane.Alt * CurrentState.multiplieralt).ToString("0") + " " + CurrentState.AltUnit);
                    sb.AppendLine("Speed: " + (hoveredPlane.Speed / 100.0 * CurrentState.multiplierspeed).ToString("0") + " " + CurrentState.SpeedUnit);
                    sb.AppendLine("Heading: " + hoveredPlane.Heading.ToString("0") + "°");

                    // Distance
                    string distanceStr;
                    if (hoveredDistance > 1000)
                        distanceStr = (hoveredDistance / 1000).ToString("0.#") + " km";
                    else
                        distanceStr = (hoveredDistance * CurrentState.multiplierdist).ToString("0") + " " + CurrentState.DistanceUnit;
                    sb.AppendLine("Distance: " + distanceStr);

                    // Altitude delta
                    double ownAlt = MainV2.comPort?.MAV?.cs?.alt ?? 0;
                    double altDelta = (hoveredPlane.Alt - ownAlt) * CurrentState.multiplieralt;
                    string altDeltaStr = (altDelta >= 0 ? "+" : "") + altDelta.ToString("0");
                    sb.AppendLine("Alt Delta: " + altDeltaStr + " " + CurrentState.AltUnit);

                    // Collision threat level
                    if (hoveredPlane.ThreatLevel != MAVLink.MAV_COLLISION_THREAT_LEVEL.NONE)
                        sb.AppendLine("Collision risk: " + (hoveredPlane.ThreatLevel == MAVLink.MAV_COLLISION_THREAT_LEVEL.LOW ? "Warning" : "Danger"));

                    _adsbToolTip.Show(sb.ToString().TrimEnd(), this, mouseX + 15, mouseY + 15);
                }
            }
            else
            {
                if (_lastHoveredADSB != null)
                {
                    _lastHoveredADSB = null;
                    _adsbToolTip.Hide(this);
                }
            }
        }

        private void OnMouseDoubleClick(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                // Only reset the camera angle, not distance or height
                _cameraAngle = 0.0;
            }
        }

        private void OnMouseWheel(object sender, MouseEventArgs e)
        {
            // Only adjust camera distance when connected
            if (!IsVehicleConnected)
                return; // Ignore scroll wheel in disconnected state

            // Adjust camera distance with scroll wheel (temporary, not saved)
            // Scroll up (positive delta) = move camera closer (decrease distance)
            // Scroll down (negative delta) = move camera further (increase distance)
            double adjustment = -e.Delta / 1200.0; // 0.1 per scroll notch (120 units per notch)
            _cameraDist = Math.Max(0.1, Math.Min(10.0, _cameraDist + adjustment));
        }

        int[] viewport = new int[4];
        Matrix4 modelMatrix = Matrix4.Identity;
        private Matrix4 projMatrix = Matrix4.Identity;

        public PointLatLngAlt getMousePos(int x, int y)
        {
            //https://gamedev.stackexchange.com/questions/103483/opentk-ray-picking
            var _start = UnProject(new Vector3(x, y, 0.0f), projMatrix, modelMatrix,
                new Size(viewport[2], viewport[3]));
            var _end = UnProject(new Vector3(x, y, 1), projMatrix, modelMatrix, new Size(viewport[2], viewport[3]));
            var pos = new utmpos(utmcenter[0] + _end.X, utmcenter[1] + _end.Y, utmzone);
            var plla = pos.ToLLA();
            plla.Alt = _end.Z;
            var camera = new utmpos(utmcenter[0] + cameraX, utmcenter[1] + cameraY, utmzone).ToLLA();
            camera.Alt = cameraZ;
            var point = srtm.getIntersectionWithTerrain(camera, plla);
            return point;
        }

        private void OnClick(object sender, EventArgs e)
        {
            //utmzone = 0;
            //this.LocationCenter = LocationCenter.newpos(0, 0.001);
        }

        public static Vector3 UnProject(Vector3 mouse, Matrix4 projection, Matrix4 view, Size viewport)
        {
            Vector4 vec;
            vec.X = 2.0f * mouse.X / (float) viewport.Width - 1;
            vec.Y = -(2.0f * mouse.Y / (float) viewport.Height - 1);
            vec.Z = mouse.Z;
            vec.W = 1.0f;
            Matrix4 viewInv = Matrix4.Invert(view);
            Matrix4 projInv = Matrix4.Invert(projection);
            Vector4.Transform(ref vec, ref projInv, out vec);
            Vector4.Transform(ref vec, ref viewInv, out vec);
            if (vec.W > 0.000001f || vec.W < -0.000001f)
            {
                vec.X /= vec.W;
                vec.Y /= vec.W;
                vec.Z /= vec.W;
            }

            return vec.Xyz;
        }

        ~Map3D()
        {
            foreach (var tileInfo in textureid)
            {
                try
                {
                    tileInfo.Value.Cleanup();
                }
                catch
                {
                }
            }
        }

        public void Deactivate()
        {
            timer1?.Stop();
            started = false;

            foreach (var tileInfo in textureid)
            {
                try
                {
                    tileInfo.Value.Cleanup();
                }
                catch
                {
                }
            }

            textureid.Clear();
        }

        public void Activate()
        {
            if (!started)
            {
                timer1?.Start();
                started = true;
            }

            // Reset Kalman filters so position/rotation jumps to current values
            // instead of slowly interpolating from stale state
            ResetKalmanFilters();

            this.Invalidate();
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                // Save camera position for next time
                try
                {
                    Settings.Instance["map3d_camera_dist"] = _cameraDist.ToString();
                    Settings.Instance["map3d_camera_height"] = _cameraHeight.ToString();
                    Settings.Instance["map3d_camera_angle"] = _cameraAngle.ToString();
                    Settings.Instance.Save();
                }
                catch { }

                _stopRequested = true;
                try { _tileTaskQueue.CompleteAdding(); } catch { }
                timer1?.Stop();
                timer1?.Dispose();

                try
                {
                    _imageloaderThread?.Join(200);
                }
                catch { }

                try
                {
                    _imageLoaderWindow?.Close();
                    _imageLoaderWindow?.Dispose();
                }
                catch
                {
                }

                _imageLoaderWindow = null;

                // Clean up textures
                Deactivate();

                // Clean up the image loader thread context
                try
                {
                    IMGContext?.Dispose();
                }
                catch { }
            }
            base.Dispose(disposing);
        }

        public Vector3 Normal(Vector3 a, Vector3 b, Vector3 c)
        {
            var dir = Vector3.Cross(b - a, c - a);
            var norm = Vector3.Normalize(dir);
            return norm;
        }


        private Dictionary<string, int> wpLabelTextures = new Dictionary<string, int>();

        private int getWpLabelTexture(string label)
        {
            if (wpLabelTextures.ContainsKey(label))
                return wpLabelTextures[label];

            // Create a bitmap with the waypoint label
            int size = 128;
            using (Bitmap bmp = new Bitmap(size, size, System.Drawing.Imaging.PixelFormat.Format32bppArgb))
            {
                using (Graphics g = Graphics.FromImage(bmp))
                {
                    g.Clear(Color.Transparent);
                    g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
                    g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.AntiAlias;

                    using (Font font = new Font("Arial", 48, FontStyle.Bold))
                    using (StringFormat sf = new StringFormat())
                    {
                        sf.Alignment = StringAlignment.Center;
                        sf.LineAlignment = StringAlignment.Center;

                        // Draw white text with black outline
                        RectangleF rect = new RectangleF(0, 0, size, size);

                        // Draw outline
                        using (System.Drawing.Drawing2D.GraphicsPath path = new System.Drawing.Drawing2D.GraphicsPath())
                        {
                            path.AddString(label, font.FontFamily, (int)font.Style, font.Size,
                                rect, sf);
                            using (Pen outlinePen = new Pen(Color.Black, 6))
                            {
                                outlinePen.LineJoin = System.Drawing.Drawing2D.LineJoin.Round;
                                g.DrawPath(outlinePen, path);
                            }
                            g.FillPath(Brushes.White, path);
                        }
                    }
                }

                int texture = generateTexture(new Bitmap(bmp));
                wpLabelTextures[label] = texture;
                return texture;
            }
        }


        private void DrawPlane(Matrix4 projMatrix, Matrix4 viewMatrix)
        {
            if (!_settingsLoaded) return;

            if (!_stlLoader.EnsureLoaded() || _stlLoader.Vertices == null || _stlLoader.VertexCount == 0)
                return;

            GL.Enable(EnableCap.DepthTest);
            GL.DepthMask(true);
            GL.Disable(EnableCap.CullFace); // Show both sides

            // STL is in millimeters, convert to meters (divide by 1000) then apply user scale multiplier
            float scale = _planeScaleMultiplier / 1000f;

            // Create model matrix for the plane
            // Build in order: Scale -> Rotate -> Translate
            var planeModelMatrix = Matrix4.CreateScale(scale);

            // Rotate: roll, pitch, then yaw (negate pitch and yaw for correct direction)
            planeModelMatrix = Matrix4.Mult(planeModelMatrix, Matrix4.CreateRotationY((float)MathHelper.Radians(_planeRoll)));
            planeModelMatrix = Matrix4.Mult(planeModelMatrix, Matrix4.CreateRotationX((float)MathHelper.Radians(_planePitch)));
            planeModelMatrix = Matrix4.Mult(planeModelMatrix, Matrix4.CreateRotationZ((float)MathHelper.Radians(-_planeYaw)));

            // Translate to position
            planeModelMatrix = Matrix4.Mult(planeModelMatrix, Matrix4.CreateTranslation((float)_planeDrawX, (float)_planeDrawY, (float)_planeDrawZ));

            // Combine model with view matrix (like other objects in the scene)
            var modelViewMatrix = Matrix4.Mult(planeModelMatrix, viewMatrix);

            // Use Lines shader which supports vertex colors for shading
            GL.UseProgram(Lines.Program);

            // set matrices
            GL.UniformMatrix4(Lines.modelViewSlot, 1, false, ref modelViewMatrix.Row0.X);
            GL.UniformMatrix4(Lines.projectionSlot, 1, false, ref projMatrix.Row0.X);

            GL.EnableVertexAttribArray(Lines.positionSlot);
            GL.EnableVertexAttribArray(Lines.colorSlot);

            // Light direction in model space (sun from above-right-front)
            // We need to transform this by inverse of model rotation to get consistent lighting
            float lightX = 0.4f, lightY = 0.6f, lightZ = 0.7f;
            float lightLen = (float)Math.Sqrt(lightX * lightX + lightY * lightY + lightZ * lightZ);
            lightX /= lightLen; lightY /= lightLen; lightZ /= lightLen;

            // Build vertex array with per-vertex lighting
            var planeVerts = new List<Vertex>();
            var vertices = _stlLoader.Vertices;
            var normals = _stlLoader.Normals;
            for (int i = 0; i < vertices.Count; i += 3)
            {
                float vx = vertices[i];
                float vy = vertices[i + 1];
                float vz = vertices[i + 2];

                // Get normal for this vertex
                float nx = normals[i];
                float ny = normals[i + 1];
                float nz = normals[i + 2];

                // Normalize
                float nlen = (float)Math.Sqrt(nx * nx + ny * ny + nz * nz);
                if (nlen > 0.001f) { nx /= nlen; ny /= nlen; nz /= nlen; }

                // Compute diffuse lighting (N dot L)
                float diffuse = Math.Max(0, nx * lightX + ny * lightY + nz * lightZ);

                // Ambient + diffuse lighting
                float ambient = 0.3f;
                float light = ambient + diffuse * 0.7f;
                light = Math.Min(1.0f, light);

                // Apply plane color with shading
                float r = (_planeColor.R / 255f) * light;
                float g = (_planeColor.G / 255f) * light;
                float b = (_planeColor.B / 255f) * light;
                planeVerts.Add(new Vertex(vx, vy, vz, r, g, b, 1.0, 0, 0));
            }

            // Create temporary VBO
            int vbo;
            GL.GenBuffers(1, out vbo);
            GL.BindBuffer(BufferTarget.ArrayBuffer, vbo);
            GL.BufferData(BufferTarget.ArrayBuffer, planeVerts.Count * Vertex.Stride, planeVerts.ToArray(), BufferUsageHint.StreamDraw);

            GL.VertexAttribPointer(Lines.positionSlot, 3, VertexAttribPointerType.Float, false, Vertex.Stride, IntPtr.Zero);
            GL.VertexAttribPointer(Lines.colorSlot, 4, VertexAttribPointerType.Float, false, Vertex.Stride, (IntPtr)(sizeof(float) * 3));

            GL.DrawArrays(BeginMode.Triangles, 0, _stlLoader.VertexCount);

            GL.DisableVertexAttribArray(Lines.positionSlot);
            GL.DisableVertexAttribArray(Lines.colorSlot);

            // Cleanup
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.DeleteBuffer(vbo);
        }

        private void DrawHeadingLines(Matrix4 projMatrix, Matrix4 viewMatrix)
        {
            // Common pitch calculation for all lines (positive pitch = nose up = higher Z)
            double pitchRad = MathHelper.Radians(_planePitch);

            // Heading line (red) - uses Kalman-filtered yaw for smooth movement, includes pitch
            if (_showHeadingLine)
            {
                double headingRad = MathHelper.Radians(_planeYaw);
                // Horizontal length is shortened by pitch angle
                double horizontalLength = HEADING_LINE_LENGTH * Math.Cos(pitchRad);
                double headingEndX = _planeDrawX + Math.Sin(headingRad) * horizontalLength;
                double headingEndY = _planeDrawY + Math.Cos(headingRad) * horizontalLength;
                // Z changes based on pitch (positive pitch = nose up = higher Z)
                double headingEndZ = _planeDrawZ + HEADING_LINE_LENGTH * Math.Sin(pitchRad);

                _headingLine?.Dispose();
                _headingLine = new Lines();
                _headingLine.Width = 1.5f;
                _headingLine.Add(_planeDrawX, _planeDrawY, _planeDrawZ, 1, 0, 0, 1);
                _headingLine.Add(headingEndX, headingEndY, headingEndZ, 1, 0, 0, 1);
                _headingLine.Draw(projMatrix, viewMatrix);
            }

            // Nav bearing line (orange) - draws from plane to target waypoint or nav_bearing direction
            if (_showNavBearingLine)
            {
                double navEndX, navEndY, navEndZ;
                PointLatLngAlt targetWp = null;

                // Only connect to actual waypoint in Auto, Guided, or RTL modes
                // In other modes, just draw a directional line like the 2D map does
                var mode = MainV2.comPort?.MAV?.cs?.mode?.ToLower() ?? "";
                bool isNavigatingMode = mode == "auto" || mode == "guided" || mode == "rtl" ||
                                        mode == "land" || mode == "smart_rtl";

                if (isNavigatingMode)
                {
                    if (mode == "guided" && MainV2.comPort?.MAV?.GuidedMode.x != 0)
                    {
                        // In guided mode, target is the guided waypoint
                        targetWp = new PointLatLngAlt(MainV2.comPort.MAV.GuidedMode)
                            { Alt = MainV2.comPort.MAV.GuidedMode.z + MainV2.comPort.MAV.cs.HomeAlt };
                    }
                    else if (mode == "rtl" || mode == "land" || mode == "smart_rtl")
                    {
                        // In RTL/Land modes, target is Home
                        var waypointList = GetWaypointListFromMAV();
                        targetWp = waypointList?.FirstOrDefault(p => p != null && p.Tag == "H");
                    }
                    else if (mode == "auto")
                    {
                        // Auto mode - use current waypoint number
                        int wpno = (int)(MainV2.comPort?.MAV?.cs?.wpno ?? 0);
                        var waypointList = GetWaypointListFromMAV().Where(a => a != null).ToList();
                        if (waypointList != null && wpno > 0 && wpno < waypointList.Count)
                            targetWp = waypointList[wpno];
                    }
                }

                if (targetWp != null && targetWp.Lat != 0 && targetWp.Lng != 0)
                {
                    // Calculate position exactly as waypoint markers are drawn
                    var co = convertCoords(targetWp);
                    var targetTerrainAlt = srtm.getAltitude(targetWp.Lat, targetWp.Lng).alt;
                    navEndX = co[0];
                    navEndY = co[1];

                    // Home target should be at terrain level
                    // Guided target (line 922) already has HomeAlt added, so it's absolute
                    if (targetWp.Tag == "H")
                    {
                        navEndZ = targetTerrainAlt;
                    }
                    else if (mode == "guided")
                    {
                        // Guided target: use terrain + GuidedMode.z (matches G marker rendering)
                        var guidedRelativeAlt = MainV2.comPort?.MAV?.GuidedMode.z ?? 0;
                        navEndZ = targetTerrainAlt + guidedRelativeAlt;
                    }
                    else
                    {
                        navEndZ = co[2] + targetTerrainAlt;
                    }
                }
                else
                {
                    // Not in navigation mode or no target: use nav_bearing direction with fixed length and pitch
                    double rawNavBearing = MainV2.comPort?.MAV?.cs?.nav_bearing ?? 0;
                    double filteredNavBearing = _kalmanNavBearing.UpdateAngle(rawNavBearing);
                    double navBearingRad = MathHelper.Radians(filteredNavBearing);
                    double horizontalLength = HEADING_LINE_LENGTH * Math.Cos(pitchRad);
                    navEndX = _planeDrawX + Math.Sin(navBearingRad) * horizontalLength;
                    navEndY = _planeDrawY + Math.Cos(navBearingRad) * horizontalLength;
                    navEndZ = _planeDrawZ + HEADING_LINE_LENGTH * Math.Sin(pitchRad);
                }

                _navBearingLine?.Dispose();
                _navBearingLine = new Lines();
                _navBearingLine.Width = 1.5f;
                _navBearingLine.Add(_planeDrawX, _planeDrawY, _planeDrawZ, 1, 0.5f, 0, 1);
                _navBearingLine.Add(navEndX, navEndY, navEndZ, 1, 0.5f, 0, 1);
                _navBearingLine.Draw(projMatrix, viewMatrix);
            }

            // GPS heading line (black)
            if (_showGpsHeadingLine)
            {
                double rawGpsHeading = MainV2.comPort?.MAV?.cs?.groundcourse ?? 0;
                double filteredGpsHeading = _kalmanGpsHeading.UpdateAngle(rawGpsHeading);
                double gpsRad = MathHelper.Radians(filteredGpsHeading);
                double gpsEndX = _planeDrawX + Math.Sin(gpsRad) * HEADING_LINE_LENGTH;
                double gpsEndY = _planeDrawY + Math.Cos(gpsRad) * HEADING_LINE_LENGTH;

                _gpsHeadingLine?.Dispose();
                _gpsHeadingLine = new Lines();
                _gpsHeadingLine.Width = 1.5f;
                _gpsHeadingLine.Add(_planeDrawX, _planeDrawY, _planeDrawZ, 0, 0, 0, 1);
                _gpsHeadingLine.Add(gpsEndX, gpsEndY, _planeDrawZ, 0, 0, 0, 1);
                _gpsHeadingLine.Draw(projMatrix, viewMatrix);
            }

            // Turn radius arc (hot pink) - shows predicted turn path based on current bank angle
            if (_showTurnRadius)
            {
                float radius = (float)(MainV2.comPort?.MAV?.cs?.radius ?? 0);

                if (Math.Abs(radius) > 1)
                {
                    double alpha = (TURN_RADIUS_ARC_LENGTH / Math.Abs(radius)) * MathHelper.rad2deg;
                    if (alpha > 180) alpha = 180;

                    // Calculate center of turn circle perpendicular to travel direction
                    // Use the same filtered GPS heading for consistency
                    double rawGpsCourse = MainV2.comPort?.MAV?.cs?.groundcourse ?? 0;
                    double filteredGpsCourse = _kalmanGpsHeading.UpdateAngle(rawGpsCourse);
                    double cogRad = MathHelper.Radians(filteredGpsCourse);
                    double perpAngle = cogRad + (radius > 0 ? Math.PI / 2 : -Math.PI / 2);

                    // Apply pitch to horizontal distances
                    double horizontalRadius = Math.Abs(radius) * Math.Cos(pitchRad);
                    double centerX = _planeDrawX + Math.Sin(perpAngle) * horizontalRadius;
                    double centerY = _planeDrawY + Math.Cos(perpAngle) * horizontalRadius;
                    double startAngle = Math.Atan2(_planeDrawX - centerX, _planeDrawY - centerY);

                    _turnRadiusLine?.Dispose();
                    _turnRadiusLine = new Lines();
                    _turnRadiusLine.Width = 2f;

                    // HotPink color
                    float r = 1.0f;
                    float g = 105f / 255f;
                    float b = 180f / 255f;

                    double alphaRad = MathHelper.Radians(alpha);
                    double angleStep = alphaRad / TURN_RADIUS_SEGMENTS;
                    double direction = radius > 0 ? 1 : -1;

                    double prevX = _planeDrawX;
                    double prevY = _planeDrawY;
                    double prevZ = _planeDrawZ;

                    // Calculate Z change per segment based on pitch and arc length
                    double arcLengthPerSegment = Math.Abs(radius) * angleStep;
                    double zChangePerSegment = arcLengthPerSegment * Math.Sin(pitchRad);

                    for (int i = 1; i <= TURN_RADIUS_SEGMENTS; i++)
                    {
                        double angle = startAngle + direction * angleStep * i;
                        double x = centerX + Math.Sin(angle) * horizontalRadius;
                        double y = centerY + Math.Cos(angle) * horizontalRadius;
                        double z = _planeDrawZ + zChangePerSegment * i;

                        _turnRadiusLine.Add(prevX, prevY, prevZ, r, g, b, 1);
                        _turnRadiusLine.Add(x, y, z, r, g, b, 1);

                        prevX = x;
                        prevY = y;
                        prevZ = z;
                    }

                    _turnRadiusLine.Draw(projMatrix, viewMatrix);
                }
            }
        }

        private void DrawTrail(Matrix4 projMatrix, Matrix4 viewMatrix)
        {
            if (!_showTrail || MainV2.comPort?.MAV?.cs?.armed != true || _trailPoints.Count < 2)
                return;

            _trailLine?.Dispose();
            _trailLine = new Lines();
            _trailLine.Width = 5f;

            // MidnightBlue color with alpha (matches 2D map GMapRoute default)
            float r = 25f / 255f;
            float g = 25f / 255f;
            float b = 112f / 255f;
            float a = 144f / 255f;

            // Convert trail points to relative coordinates (oldest to newest)
            var rawPoints = new List<double[]>();
            for (int i = 0; i < _trailPoints.Count; i++)
            {
                var pt = _trailPoints[i];
                rawPoints.Add(new double[] { pt[0] - utmcenter[0], pt[1] - utmcenter[1], pt[2] });
            }

            if (rawPoints.Count < 2)
            {
                _trailLine.Draw(projMatrix, viewMatrix);
                return;
            }

            // Simple moving average smoothing
            var smoothed = PreSmoothPoints(rawPoints, 20);

            // Draw smoothed points from oldest to newest
            foreach (var pt in smoothed)
            {
                _trailLine.Add(pt[0], pt[1], pt[2], r, g, b, a);
            }

            // End at current plane position
            _trailLine.Add(_planeDrawX, _planeDrawY, _planeDrawZ, r, g, b, a);

            _trailLine.Draw(projMatrix, viewMatrix);
        }

        /// <summary>
        /// Pre-smooths raw points with a moving average to eliminate noise/oscillations.
        /// Uses a larger window for Z (altitude) since it tends to be noisier.
        /// </summary>
        private List<double[]> PreSmoothPoints(List<double[]> points, int windowSizeXY, int windowSizeZ = -1)
        {
            if (windowSizeZ < 0) windowSizeZ = windowSizeXY * 3; // Default Z window is 3x XY

            if (points.Count < Math.Max(windowSizeXY, windowSizeZ))
                return points;

            var result = new List<double[]>();
            int halfWindowXY = windowSizeXY / 2;
            int halfWindowZ = windowSizeZ / 2;

            for (int i = 0; i < points.Count; i++)
            {
                // Smooth XY with standard window
                double sumX = 0, sumY = 0;
                int countXY = 0;
                int startXY = Math.Max(0, i - halfWindowXY);
                int endXY = Math.Min(points.Count - 1, i + halfWindowXY);

                for (int j = startXY; j <= endXY; j++)
                {
                    sumX += points[j][0];
                    sumY += points[j][1];
                    countXY++;
                }

                // Smooth Z with larger window for less altitude jitter
                double sumZ = 0;
                int countZ = 0;
                int startZ = Math.Max(0, i - halfWindowZ);
                int endZ = Math.Min(points.Count - 1, i + halfWindowZ);

                for (int j = startZ; j <= endZ; j++)
                {
                    sumZ += points[j][2];
                    countZ++;
                }

                result.Add(new double[] { sumX / countXY, sumY / countXY, sumZ / countZ });
            }

            return result;
        }

        /// <summary>
        /// Adaptively samples points based on path curvature.
        /// Keeps more points in areas with sharp turns, fewer in straight sections.
        /// </summary>
        private List<double[]> AdaptiveSample(List<double[]> points, double deviationThreshold, double angleThreshold)
        {
            if (points.Count < 3)
                return points;

            var result = new List<double[]>();
            result.Add(points[0]); // Always keep first point

            int lastKeptIndex = 0;
            double angleThresholdRad = angleThreshold * Math.PI / 180.0;

            for (int i = 1; i < points.Count - 1; i++)
            {
                var lastKept = points[lastKeptIndex];
                var current = points[i];
                var next = points[i + 1];

                // Check 1: Deviation from straight line (lastKept -> next)
                double deviation = PointToLineDistance(current, lastKept, next);

                // Check 2: Angle change at this point
                double angle = 0;
                if (i > 0)
                {
                    var prev = points[i - 1];
                    angle = AngleBetweenSegments(prev, current, next);
                }

                // Check 3: Distance from last kept point (ensure minimum sampling)
                double distFromLastKept = Distance3D(lastKept, current);

                // Keep point if:
                // - Deviation exceeds threshold (path curves away from straight line)
                // - Angle change exceeds threshold (sharp turn)
                // - Distance exceeds max spacing (ensure some minimum resolution)
                bool keepPoint = deviation > deviationThreshold ||
                                 angle > angleThresholdRad ||
                                 distFromLastKept > 50; // Max 50m between control points

                if (keepPoint)
                {
                    result.Add(current);
                    lastKeptIndex = i;
                }
            }

            result.Add(points[points.Count - 1]); // Always keep last point (plane position)
            return result;
        }

        /// <summary>
        /// Calculates perpendicular distance from point to line segment.
        /// </summary>
        private double PointToLineDistance(double[] point, double[] lineStart, double[] lineEnd)
        {
            double dx = lineEnd[0] - lineStart[0];
            double dy = lineEnd[1] - lineStart[1];
            double dz = lineEnd[2] - lineStart[2];
            double lineLengthSq = dx * dx + dy * dy + dz * dz;

            if (lineLengthSq < 0.0001)
                return Distance3D(point, lineStart);

            // Project point onto line
            double t = ((point[0] - lineStart[0]) * dx +
                        (point[1] - lineStart[1]) * dy +
                        (point[2] - lineStart[2]) * dz) / lineLengthSq;
            t = Math.Max(0, Math.Min(1, t));

            double projX = lineStart[0] + t * dx;
            double projY = lineStart[1] + t * dy;
            double projZ = lineStart[2] + t * dz;

            return Math.Sqrt(
                Math.Pow(point[0] - projX, 2) +
                Math.Pow(point[1] - projY, 2) +
                Math.Pow(point[2] - projZ, 2));
        }

        /// <summary>
        /// Calculates angle between two segments at a point (in radians).
        /// </summary>
        private double AngleBetweenSegments(double[] p1, double[] p2, double[] p3)
        {
            // Vector from p1 to p2
            double v1x = p2[0] - p1[0];
            double v1y = p2[1] - p1[1];
            double v1z = p2[2] - p1[2];

            // Vector from p2 to p3
            double v2x = p3[0] - p2[0];
            double v2y = p3[1] - p2[1];
            double v2z = p3[2] - p2[2];

            double dot = v1x * v2x + v1y * v2y + v1z * v2z;
            double len1 = Math.Sqrt(v1x * v1x + v1y * v1y + v1z * v1z);
            double len2 = Math.Sqrt(v2x * v2x + v2y * v2y + v2z * v2z);

            if (len1 < 0.0001 || len2 < 0.0001)
                return 0;

            double cosAngle = dot / (len1 * len2);
            cosAngle = Math.Max(-1, Math.Min(1, cosAngle)); // Clamp for numerical stability

            return Math.Acos(cosAngle); // Returns angle in radians (0 = straight, PI = 180° turn)
        }

        private double Distance3D(double[] a, double[] b)
        {
            return Math.Sqrt(
                Math.Pow(a[0] - b[0], 2) +
                Math.Pow(a[1] - b[1], 2) +
                Math.Pow(a[2] - b[2], 2));
        }

        /// <summary>
        /// Generates a smooth curve using Catmull-Rom spline interpolation.
        /// </summary>
        private List<double[]> CatmullRomSpline(List<double[]> points, int segments)
        {
            if (points.Count < 2)
                return points;

            var result = new List<double[]>();
            result.Add(points[0]);

            for (int i = 0; i < points.Count - 1; i++)
            {
                var p0 = points[Math.Max(0, i - 1)];
                var p1 = points[i];
                var p2 = points[i + 1];
                var p3 = points[Math.Min(points.Count - 1, i + 2)];

                for (int j = 1; j <= segments; j++)
                {
                    double t = (double)j / segments;
                    result.Add(CatmullRomPoint(p0, p1, p2, p3, t));
                }
            }

            return result;
        }

        private double[] CatmullRomPoint(double[] p0, double[] p1, double[] p2, double[] p3, double t)
        {
            double t2 = t * t;
            double t3 = t2 * t;

            double b0 = -0.5 * t3 + t2 - 0.5 * t;
            double b1 = 1.5 * t3 - 2.5 * t2 + 1.0;
            double b2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
            double b3 = 0.5 * t3 - 0.5 * t2;

            return new double[]
            {
                b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
                b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1],
                b0 * p0[2] + b1 * p1[2] + b2 * p2[2] + b3 * p3[2]
            };
        }

        /// <summary>
        /// Draws ADSB aircraft as circles on the 3D map.
        /// ADSB altitude is MSL (barometric), so no terrain adjustment needed.
        /// Circle diameter is fixed at 250m (25m for grounded aircraft).
        /// Color based on distance from own aircraft:
        /// Red if within 5km, Yellow if within 20km, Green if > 50km, interpolated between.
        /// Grounded aircraft are drawn as light gray circles.
        /// Circles are billboarded to always face the camera.
        /// Drawn in reverse order of distance (farthest first) for proper depth ordering.
        /// </summary>
        private void DrawADSB(Matrix4 projMatrix, Matrix4 viewMatrix)
        {
            double circleRadius = _adsbCircleSize / 2.0;
            double groundedCircleRadius = circleRadius / 5.0;

            // Use vehicle location when connected, otherwise use 2D map center position
            var ownPosition = IsVehicleConnected
                ? (MainV2.comPort?.MAV?.cs?.Location ?? _center)
                : _center;

            _adsbScreenPositions.Clear();

            var planeList = new List<Tuple<adsb.PointLatLngAltHdg, double>>();

            lock (MainV2.instance.adsblock)
            {
                foreach (var kvp in MainV2.instance.adsbPlanes)
                {
                    var plane = kvp.Value;
                    if (plane == null)
                        continue;

                    if (plane.Time < DateTime.Now.AddSeconds(-30))
                        continue;

                    if (plane.Lat == 0 && plane.Lng == 0)
                        continue;

                    var plla = new PointLatLngAlt(plane.Lat, plane.Lng, plane.Alt);
                    double distanceToOwn = ownPosition.GetDistance(plla);

                    if (distanceToOwn > ADSB_MAX_DISTANCE)
                        continue;

                    planeList.Add(Tuple.Create(plane, distanceToOwn));
                }
            }

            // Sort by distance descending (farthest first) so closer planes render on top
            planeList.Sort((a, b) => b.Item2.CompareTo(a.Item2));

            var circleVertices = new List<float>();

            foreach (var item in planeList)
            {
                var plane = item.Item1;
                double distanceToOwn = item.Item2;

                bool isGrounded = plane.IsOnGround;
                double radius = isGrounded ? groundedCircleRadius : circleRadius;

                var plla = new PointLatLngAlt(plane.Lat, plane.Lng, plane.Alt);
                var co = convertCoords(plla);

                // For grounded aircraft, ensure they're visible above terrain
                if (isGrounded)
                {
                    var terrainAlt = srtm.getAltitude(plane.Lat, plane.Lng).alt;
                    if (co[2] < terrainAlt + 10)
                    {
                        co[2] = terrainAlt + 10; // Place at least 10m above terrain
                    }
                }

                var color = GetADSBDistanceColor(distanceToOwn, isGrounded);

                var billboard = CalculateBillboardOrientation(co[0], co[1], co[2], cameraX, cameraY, cameraZ);
                if (!billboard.HasValue)
                    continue;

                // Calculate distance to camera for screen position calculation
                double dx = co[0] - cameraX;
                double dy = co[1] - cameraY;
                double dz = co[2] - cameraZ;
                double distanceToCamera = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                // Store screen position for hit testing
                var worldPos = new Vector4((float)co[0], (float)co[1], (float)co[2], 1.0f);
                var clipPos = Vector4.Transform(worldPos, viewMatrix * projMatrix);
                if (clipPos.W > 0)
                {
                    float ndcX = clipPos.X / clipPos.W;
                    float ndcY = clipPos.Y / clipPos.W;
                    float screenX = (ndcX + 1.0f) * 0.5f * Width;
                    float screenY = (1.0f - ndcY) * 0.5f * Height;

                    double fovRad = _cameraFOV * MathHelper.deg2rad;
                    float screenRadius = (float)((radius / distanceToCamera) * (Height / 2.0) / Math.Tan(fovRad / 2.0));

                    _adsbScreenPositions.Add(new ADSBScreenPosition
                    {
                        ScreenX = screenX,
                        ScreenY = screenY,
                        Radius = Math.Max(screenRadius, 20),
                        PlaneData = plane,
                        DistanceToOwn = distanceToOwn
                    });
                }

                var (rightX, rightY, rightZ, upX, upY, upZ) = billboard.Value;
                AddBillboardCircleVertices(
                    co[0], co[1], co[2],
                    radius, ADSB_CIRCLE_SEGMENTS,
                    color.r, color.g, color.b, color.a,
                    rightX, rightY, rightZ, upX, upY, upZ,
                    circleVertices);
            }

            if (circleVertices.Count > 0)
            {
                DrawADSBCircles(projMatrix, viewMatrix, circleVertices.ToArray());
            }
        }

        /// <summary>
        /// Draws ADSB circles using GL.Lines primitive (not LineStrip) so circles are not connected.
        /// </summary>
        private void DrawADSBCircles(Matrix4 projMatrix, Matrix4 viewMatrix, float[] vertices)
        {
            if (Lines.Program == 0)
                return;

            // Enable depth testing so circles are properly occluded by terrain
            GL.Enable(EnableCap.DepthTest);
            GL.DepthMask(true);
            GL.DepthFunc(DepthFunction.Lequal);

            GL.UseProgram(Lines.Program);

            GL.EnableVertexAttribArray(Lines.positionSlot);
            GL.EnableVertexAttribArray(Lines.colorSlot);

            GL.UniformMatrix4(Lines.modelViewSlot, 1, false, ref viewMatrix.Row0.X);
            GL.UniformMatrix4(Lines.projectionSlot, 1, false, ref projMatrix.Row0.X);

            GL.LineWidth(3f);

            // Create and bind vertex buffer
            int vbo;
            GL.GenBuffers(1, out vbo);
            GL.BindBuffer(BufferTarget.ArrayBuffer, vbo);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Length * sizeof(float), vertices, BufferUsageHint.DynamicDraw);

            // Stride: 7 floats per vertex (x, y, z, r, g, b, a)
            int stride = 7 * sizeof(float);
            GL.VertexAttribPointer(Lines.positionSlot, 3, VertexAttribPointerType.Float, false, stride, IntPtr.Zero);
            GL.VertexAttribPointer(Lines.colorSlot, 4, VertexAttribPointerType.Float, false, stride, (IntPtr)(3 * sizeof(float)));

            // Draw as GL.Lines (pairs of vertices form separate lines)
            GL.DrawArrays(PrimitiveType.Lines, 0, vertices.Length / 7);

            GL.DisableVertexAttribArray(Lines.positionSlot);
            GL.DisableVertexAttribArray(Lines.colorSlot);

            // Clean up
            GL.DeleteBuffers(1, ref vbo);
        }

        public PointLatLngAlt mousePosition { get; private set; }

        public Utilities.Vector3 Velocity
        {
            get { return _velocity; }
            set { _velocity = value; }
        }

        private int utmzone = -999;
        private PointLatLngAlt llacenter = PointLatLngAlt.Zero;
        private double[] utmcenter = new double[2];
        private PointLatLngAlt mouseDownPos;
        private int mousex;
        private int mousey;
        private bool _isDragging = false;
        private int _dragStartX;
        private int _dragStartY;
        private bool started;
        private bool onpaintrun;
        private double[] mypos = new double[3];
        Vector3 myrpy = Vector3.UnitX;
        private bool fogon = false;
        private Lines _flightPlanLines;
        private int _flightPlanLinesCount = -1;
        private int _flightPlanLinesHash = 0;
        private readonly FpsOverlay _fpsOverlay = new FpsOverlay();
        private DateTime _centerTime;

        // Kalman filters for smooth position and rotation interpolation
        // Lower q = smoother output, higher r = trust measurements less (smoother)
        private SimpleKalmanFilter _kalmanPosX = new SimpleKalmanFilter(0.02, 1.5);
        private SimpleKalmanFilter _kalmanPosY = new SimpleKalmanFilter(0.02, 1.5);
        private SimpleKalmanFilter _kalmanPosZ = new SimpleKalmanFilter(0.01, 2.0); // altitude is typically noisier

        // Sky gradient quad
        private Lines _skyGradient;
        // Heading and nav bearing indicator lines
        private Lines _headingLine;
        private Lines _navBearingLine;
        private Lines _gpsHeadingLine;
        private Lines _turnRadiusLine;
        // Rotation filters - extra smooth since jerky rotation is very noticeable
        private SimpleKalmanFilter _kalmanRoll = new SimpleKalmanFilter(0.015, 2.0);
        private SimpleKalmanFilter _kalmanPitch = new SimpleKalmanFilter(0.015, 2.0);
        private SimpleKalmanFilter _kalmanYaw = new SimpleKalmanFilter(0.015, 2.5); // yaw tends to be noisiest
        // Filters for heading indicator lines
        private SimpleKalmanFilter _kalmanNavBearing = new SimpleKalmanFilter(0.015, 2.0);
        private SimpleKalmanFilter _kalmanGpsHeading = new SimpleKalmanFilter(0.015, 2.0);
        private bool _kalmanInitialized = false;
        private HashSet<tileInfo> _lastTilesToRender = new HashSet<tileInfo>();

        double[] convertCoords(PointLatLngAlt plla)
        {
            var utm = plla.ToUTM(utmzone);
            Array.Resize(ref utm, 3);
            utm[0] -= utmcenter[0];
            utm[1] -= utmcenter[1];
            utm[2] = plla.Alt;
            return new[] {utm[0], utm[1], utm[2]};
        }

        protected override void OnPaint(System.Windows.Forms.PaintEventArgs e)
        {
            if (!started)
                timer1.Start();
            started = true;
            onpaintrun = true;
            try
            {
                base.OnPaint(e);
            }
            catch
            {
                return;
            }

            Utilities.Extensions.ProtectReentry(doPaint);
        }

        public void doPaint()
        {
            DateTime start = DateTime.Now;
            if (this.DesignMode)
                return;
            var beforewait = DateTime.Now;
            if (textureSemaphore.Wait(1) == false)
                return;
            var afterwait = DateTime.Now;
            try
            {
                DrainReadyTiles(3);
                DrainDisposals(2);
                double heightscale = 1; //(step/90.0)*5;
                var campos = convertCoords(_center);

                if (!IsVehicleConnected)
                {
                    ResetKalmanFilters();

                    // Vehicle is disconnected - use 2D map's center position and place camera 100m above terrain
                    PointLatLng? map2DPosition = null;
                    try
                    {
                        map2DPosition = GCSViews.FlightData.instance?.gMapControl1?.Position;
                    }
                    catch { }

                    if (map2DPosition != null && map2DPosition.Value.Lat != 0 && map2DPosition.Value.Lng != 0)
                    {
                        var newCenter = new PointLatLngAlt(map2DPosition.Value.Lat, map2DPosition.Value.Lng, 0);

                        // First-time initialization: if UTM zone is invalid, set LocationCenter synchronously
                        // to properly initialize the coordinate system before rendering
                        if (utmzone == -999)
                        {
                            LocationCenter = newCenter;
                            _lastMap2DPosition = newCenter;
                            _lastMap2DPositionChangeTime = DateTime.Now;
                        }

                        // Check if this is the first update since disconnecting (large distance from current center to 2D map)
                        // This ensures we immediately jump to the 2D map location on disconnect
                        bool isFirstDisconnectUpdate = _center.GetDistance(newCenter) > 1000;

                        // Check if 2D map position changed significantly
                        bool map2DPositionChanged = _lastMap2DPosition == null ||
                            (_lastMap2DPosition.Lat == 0 && _lastMap2DPosition.Lng == 0) ||
                            _lastMap2DPosition.GetDistance(newCenter) > 30;

                        if (map2DPositionChanged)
                        {
                            _lastMap2DPosition = newCenter;
                            _lastMap2DPositionChangeTime = DateTime.Now;
                        }

                        // Update center immediately on first disconnect, or after 1s debounce for normal updates
                        bool shouldUpdateCenter = _lastMap2DPosition != null &&
                            _lastMap2DPosition.Lat != 0 && _lastMap2DPosition.Lng != 0 &&
                            _center.GetDistance(_lastMap2DPosition) > 30 &&
                            (isFirstDisconnectUpdate || (DateTime.Now - _lastMap2DPositionChangeTime).TotalMilliseconds >= 1000);

                        if (shouldUpdateCenter)
                        {
                            // Check if UTM zone changed or large distance - need to reset coordinate system
                            bool needsCoordinateReset = utmzone != _lastMap2DPosition.GetUTMZone() || llacenter.GetDistance(_lastMap2DPosition) > 10000;

                            if (needsCoordinateReset)
                            {
                                // Use LocationCenter setter which handles cleanup properly
                                LocationCenter = _lastMap2DPosition;
                            }
                            else
                            {
                                // Small distance change - update directly without full reset
                                _center.Lat = _lastMap2DPosition.Lat;
                                _center.Lng = _lastMap2DPosition.Lng;
                                _center.Alt = _lastMap2DPosition.Alt;
                            }
                        }

                        // Convert to local coordinates (use current _center which may have just been updated)
                        var localPos = convertCoords(_center);

                        // Get terrain altitude at this location
                        var terrainAlt = srtm.getAltitude(_center.Lat, _center.Lng).alt;

                        // Camera position at center, 100m above terrain
                        cameraX = localPos[0];
                        cameraY = localPos[1];
                        cameraZ = terrainAlt + 100;

                        // Calculate look direction based on base heading + free-look yaw/pitch
                        // Base heading is last known plane yaw, or 0 (north) if never connected
                        double baseHeading = _planeYaw;
                        double totalYaw = baseHeading + _disconnectedCameraYaw;
                        double yawRad = MathHelper.Radians(totalYaw);
                        double pitchRad = MathHelper.Radians(_disconnectedCameraPitch);

                        // Calculate look-at point based on yaw and pitch
                        // Looking distance of 100m in the look direction
                        double lookDist = 100.0;
                        double cosPitch = Math.Cos(pitchRad);
                        lookX = cameraX + Math.Sin(yawRad) * cosPitch * lookDist;
                        lookY = cameraY + Math.Cos(yawRad) * cosPitch * lookDist;
                        lookZ = cameraZ + Math.Sin(pitchRad) * lookDist;
                    }
                } 
                else 
                {
                    campos = projectLocation(mypos);
                    // Apply Kalman filter to rotation for smooth interpolation
                    var rpy = filterRotation(this.rpy);

                    // save the state
                    mypos = campos;
                    myrpy = new OpenTK.Vector3((float)rpy.x, (float)rpy.y, (float)rpy.z);

                    // Plane position (where camera used to be)
                    _planeDrawX = campos[0];
                    _planeDrawY = campos[1];
                    _planeDrawZ = (campos[2] < srtm.getAltitude(_center.Lat, _center.Lng).alt)
                        ? (srtm.getAltitude(_center.Lat, _center.Lng).alt + 1) * heightscale
                        : _center.Alt * heightscale;

                    // Store plane rotation
                    _planeRoll = (float)rpy.X;
                    _planePitch = (float)rpy.Y;
                    _planeYaw = (float)rpy.Z;

                    // Update trail points
                    if (_showTrail && MainV2.comPort?.MAV?.cs?.armed == true && _center.Lat != 0 && _center.Lng != 0)
                    {
                        // Store absolute UTM coordinates
                        double absX = _planeDrawX + utmcenter[0];
                        double absY = _planeDrawY + utmcenter[1];
                        double absZ = _planeDrawZ;

                        // Wait for stable telemetry before recording (avoid 0-altitude initial points)
                        if (absZ < 0.5)
                        {
                            // Altitude is essentially 0, reset stability counter
                            _trailStableFrames = 0;
                        }
                        else
                        {
                            _trailStableFrames++;
                        }

                        // Only record trail points after telemetry has stabilized
                        if (_trailStableFrames >= TrailStabilityThreshold)
                        {
                            // Clear trail if UTM zone changed
                            if (_trailUtmZone != utmzone)
                            {
                                _trailPoints.Clear();
                                _trailUtmZone = utmzone;
                            }

                            // Add point every frame
                            int numTrackLength = Settings.Instance.GetInt32("NUM_tracklength", 200) * 15;
                            if (_trailPoints.Count > numTrackLength)
                                _trailPoints.RemoveRange(0, _trailPoints.Count - numTrackLength);
                            _trailPoints.Add(new double[] { absX, absY, absZ });
                        }
                    }
                    else
                    {
                        // Reset stability counter when not armed or no valid position
                        _trailStableFrames = 0;
                    }

                    if (_fpvMode)
                    {
                        // FPV mode: camera at aircraft position, looking in direction of flight
                        // Uses same filtered values as plane model (_planeYaw, _planePitch)
                        cameraX = _planeDrawX;
                        cameraY = _planeDrawY;
                        cameraZ = _planeDrawZ;

                        // Look direction from filtered yaw and pitch
                        double lookDist = 100;
                        double yawRad = MathHelper.Radians(_planeYaw);
                        double pitchRad = MathHelper.Radians(_planePitch);
                        double cosPitch = Math.Cos(pitchRad);
                        lookX = cameraX + Math.Sin(yawRad) * cosPitch * lookDist;
                        lookY = cameraY + Math.Cos(yawRad) * cosPitch * lookDist;
                        lookZ = cameraZ + Math.Sin(pitchRad) * lookDist;
                    }
                    else
                    {
                        // Normal mode: Camera orbits around plane at _cameraDist, offset by _cameraAngle from behind
                        double cameraAngleRad = MathHelper.Radians(rpy.Z + _cameraAngle + 180); // +180 to start behind plane
                        cameraX = _planeDrawX + Math.Sin(cameraAngleRad) * _cameraDist;
                        cameraY = _planeDrawY + Math.Cos(cameraAngleRad) * _cameraDist;
                        cameraZ = _planeDrawZ + _cameraHeight;

                        // Look at the plane
                        lookX = _planeDrawX;
                        lookY = _planeDrawY;
                        lookZ = _planeDrawZ;
                    }
                }
                if (!Context.IsCurrent)
                    Context.MakeCurrent(this.WindowInfo);
                /*Console.WriteLine("cam: {0} {1} {2} lookat: {3} {4} {5}", (float) cameraX, (float) cameraY, (float) cameraZ,
                    (float) lookX,
                    (float) lookY, (float) lookZ);
                  */
                modelMatrix = Matrix4.LookAt((float) cameraX, (float) cameraY, (float) cameraZ,
                    (float) lookX, (float) lookY, (float) lookZ,
                    0, 0, 1);

                if (_fpvMode && IsVehicleConnected)
                {
                    // In FPV mode, apply roll via matrix multiplication using same filtered values as plane
                    // Roll around Z axis (using filtered _planeRoll)
                    modelMatrix = Matrix4.Mult(modelMatrix, Matrix4.CreateRotationZ((float)(_planeRoll * MathHelper.deg2rad)));
                }

                // Update projection matrix based on altitude - render distance matches tile loading distance
                float renderDistance = (float)LodFarDistance;
                // Two-pass rendering: use larger near plane for terrain (better depth precision at high altitudes)
                // Plane will be rendered in second pass with 0.1f near plane
                float terrainNearPlane = _center.Alt > 500 ? 5.0f : (_center.Alt > 100 ? 1.0f : 0.1f);
                projMatrix = OpenTK.Matrix4.CreatePerspectiveFieldOfView(
                    (float) (_cameraFOV * MathHelper.deg2rad),
                    (float) Width / Height, terrainNearPlane,
                    renderDistance);

                {
                    // for unproject - updated on every draw
                    GL.GetInteger(GetPName.Viewport, viewport);
                }


                var beforeclear = DateTime.Now;
                //GL.Viewport(0, 0, Width, Height);
                // Clear depth and draw sky gradient
                GL.Clear(ClearBufferMask.DepthBufferBit | ClearBufferMask.AccumBufferBit);

                // Draw sky gradient background (uses theme colors)
                GL.Disable(EnableCap.DepthTest);
                DrawSkyGradient();

                // Enable depth testing for terrain so ADSB circles can be occluded
                GL.Enable(EnableCap.DepthTest);
                GL.DepthMask(true);
                GL.DepthFunc(DepthFunction.Lequal);

                GL.Enable(EnableCap.Blend);
                GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
                GL.BlendEquation(BlendEquationMode.FuncAdd);

                // Set fog uniforms for tile shader - fog starts at 80% of tile distance, ends at tile edge
                Color fogColor = ThemeManager.HudSkyBot.A > 0 ? ThemeManager.HudSkyBot : Color.LightBlue;
                float tileDist = (float)LodFarDistance;
                tileInfo.SetFogParams(tileDist * 0.8f, tileDist, fogColor);

                var beforedraw = DateTime.Now;

                // LOD Rendering: Render tiles with parent fallback for smooth loading
                // 1. Get the list of tiles we WANT to render (from current tileArea)
                // 2. For each wanted tile, use it if loaded, otherwise use best loaded ancestor
                // 3. Track which geographic areas are covered to avoid double-rendering

                var tilesToRender = new HashSet<tileInfo>();
                var coveredAreas = new HashSet<(long x, long y, int zoom)>();

                // Get current wanted tiles from tileArea
                List<tileZoomArea> currentTileArea;
                lock (_tileAreaLock)
                {
                    currentTileArea = tileArea.ToList();
                }

                // Process from highest zoom to lowest (prioritize detail)
                foreach (var ta in currentTileArea.OrderByDescending(t => t.zoom))
                {
                    foreach (var p in ta.points)
                    {
                        // Skip if this area is already covered by a higher zoom tile
                        if (IsAreaCovered(p.X, p.Y, ta.zoom, coveredAreas))
                            continue;

                        // Try to get the exact tile we want
                        if (textureid.TryGetValue(p, out var exactTile) && exactTile.zoom == ta.zoom && exactTile.indices.Count > 0)
                        {
                            tilesToRender.Add(exactTile);
                            MarkAreaCovered(p.X, p.Y, ta.zoom, coveredAreas);
                        }
                        else
                        {
                            // Tile not loaded - find best available ancestor (parent fallback)
                            var fallback = FindBestLoadedTile(p, ta.zoom);
                            if (fallback != null && fallback.indices.Count > 0)
                            {
                                tilesToRender.Add(fallback);
                                // Mark the fallback's area as covered (at its zoom level)
                                MarkAreaCovered(fallback.point.X, fallback.point.Y, fallback.zoom, coveredAreas);
                            }
                        }
                    }
                }

                // Also add any loaded tiles that cover areas not yet covered
                // (handles edge cases and ensures no gaps)
                foreach (var kvp in textureid)
                {
                    var tile = kvp.Value;
                    if (tile.indices.Count > 0 && !IsAreaCovered(tile.point.X, tile.point.Y, tile.zoom, coveredAreas))
                    {
                        tilesToRender.Add(tile);
                        MarkAreaCovered(tile.point.X, tile.point.Y, tile.zoom, coveredAreas);
                    }
                }

                // If nothing to render this frame, reuse last good set to prevent flicker
                if (tilesToRender.Count == 0 && _lastTilesToRender.Count > 0)
                {
                    tilesToRender = new HashSet<tileInfo>(_lastTilesToRender);
                }
                else if (tilesToRender.Count > 0)
                {
                    _lastTilesToRender = new HashSet<tileInfo>(tilesToRender);
                }

                // ============ PASS 1: TERRAIN ONLY (with larger near plane for depth precision) ============
                // Render all selected tiles (sorted by zoom for proper depth ordering)
                // Use polygon offset to prevent Z-fighting at high altitudes
                GL.Enable(EnableCap.PolygonOffsetFill);
                GL.PolygonOffset(1.0f, 1.0f);
                foreach (var tile in tilesToRender.OrderBy(t => t.zoom))
                {
                    tile.Draw(projMatrix, modelMatrix);
                }
                GL.Disable(EnableCap.PolygonOffsetFill);

                // Draw ADSB aircraft circles (in Pass 1 so they're depth-tested against terrain)
                GL.Disable(EnableCap.Texture2D);
                DrawADSB(projMatrix, modelMatrix);

                // ============ PASS 2: EVERYTHING ELSE (with small near plane) ============
                // Clear depth buffer and switch to 0.1f near plane for all other objects
                GL.Clear(ClearBufferMask.DepthBufferBit);
                var pass2ProjMatrix = OpenTK.Matrix4.CreatePerspectiveFieldOfView(
                    (float) (_cameraFOV * MathHelper.deg2rad),
                    (float) Width / Height, 0.1f,
                    renderDistance);

                var beforewps = DateTime.Now;
                GL.Enable(EnableCap.DepthTest);
                GL.Disable(EnableCap.Texture2D);

                // draw after terrain - need depth check
                {
                    GL.Enable(EnableCap.DepthTest);
                    var waypointList = GetWaypointListFromMAV();
                    var pointlistCount = waypointList.Count;
                    if (pointlistCount > 1)
                    {
                        var currentHash = ComputeWaypointHash(waypointList);
                        // Only rebuild lines if pointlist changed
                        if (_flightPlanLines == null || _flightPlanLinesCount != pointlistCount || _flightPlanLinesHash != currentHash)
                        {
                            if (_flightPlanLines != null)
                                _flightPlanLines.Dispose();
                            _flightPlanLines = new Lines();
                            _flightPlanLines.Width = 3.0f;
                            // render wps
                            foreach (var point in waypointList)
                            {
                                if (point == null)
                                    continue;
                                var co = convertCoords(point);
                                var terrainAlt = srtm.getAltitude(point.Lat, point.Lng).alt;
                                // Home is at terrain level, other waypoints are relative + terrain
                                double wpAlt = point.Tag == "H" ? terrainAlt : co[2] + terrainAlt;
                                _flightPlanLines.Add(co[0], co[1], wpAlt, 1, 1, 0, 1);
                            }
                            _flightPlanLinesCount = pointlistCount;
                            _flightPlanLinesHash = currentHash;
                        }

                        _flightPlanLines.Draw(pass2ProjMatrix, modelMatrix);
                    }
                }

                var beforewpsmarkers = DateTime.Now;
                // Draw waypoint markers (hidden if within 200ft / 61m of camera)
                {
                    if (green == 0)
                        green = generateTexture(GMap.NET.Drawing.Properties.Resources.wp_3d.ToBitmap());
                    if (greenAlt == 0)
                        greenAlt = generateTexture(GMap.NET.Drawing.Properties.Resources.wp_3d_alt.ToBitmap());

                    GL.Enable(EnableCap.DepthTest);
                    GL.DepthMask(false);
                    GL.Enable(EnableCap.Blend);
                    GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
                    GL.Enable(EnableCap.Texture2D);
                    GL.Disable(EnableCap.CullFace);

                    var waypointList = GetWaypointListFromMAV();
                    var list = waypointList.Where(a => a != null).ToList();
                    if (MainV2.comPort.MAV.cs.mode.ToLower() == "guided")
                        list.Add(new PointLatLngAlt(MainV2.comPort.MAV.GuidedMode)
                            {Alt = MainV2.comPort.MAV.GuidedMode.z + MainV2.comPort.MAV.cs.HomeAlt, Tag = "G"});
                    if (MainV2.comPort.MAV.cs.TargetLocation != PointLatLngAlt.Zero)
                        list.Add(MainV2.comPort.MAV.cs.TargetLocation);

                    // Get pointlist for wp number lookup
                    var pointlist = waypointList.Where(a => a != null).ToList();

                    foreach (var point in list.OrderByDescending((a)=> a.GetDistance(MainV2.comPort.MAV.cs.Location)))
                    {
                        if (point == null)
                            continue;
                        if (point.Lat == 0 && point.Lng == 0)
                            continue;

                        // Skip markers within 200ft of the camera/vehicle
                        var distanceToCamera = point.GetDistance(_center);
                        if (distanceToCamera < WAYPOINT_MIN_DISTANCE)
                            continue;

                        var co = convertCoords(point);
                        var terrainAlt = srtm.getAltitude(point.Lat, point.Lng).alt;
                        double wpAlt;

                        // Home marker: Place at terrain level (home is on the ground)
                        // Guided waypoint (G): Use terrain + GuidedMode.z (relative altitude above terrain)
                        // Regular waypoints: wp.z is relative, add terrain altitude
                        if (point.Tag == "H")
                        {
                            // Home position - place at terrain level (home should be on ground)
                            wpAlt = terrainAlt;
                        }
                        else if (point.Tag == "G")
                        {
                            // Guided waypoint - use terrain + the relative altitude from GuidedMode.z
                            var guidedRelativeAlt = MainV2.comPort?.MAV?.GuidedMode.z ?? 0;
                            wpAlt = terrainAlt + guidedRelativeAlt;
                        }
                        else
                        {
                            // Other waypoints are relative - add terrain altitude
                            wpAlt = co[2] + terrainAlt;
                        }

                        // Determine label first to choose correct marker texture
                        int wpIndex = pointlist.IndexOf(point);
                        string wpLabel = null;
                        if (point.Tag == "H")
                            wpLabel = "H";
                        else if (point.Tag != null && point.Tag.StartsWith("ROI"))
                            wpLabel = "R";
                        else if (IsGuidedWaypoint(point))
                            wpLabel = "G";
                        else if (wpIndex >= 0)
                            wpLabel = wpIndex.ToString();

                        // Use alt texture for non-number labels (H, R, G, etc.)
                        bool isSpecialLabel = wpLabel != null && !char.IsDigit(wpLabel[0]);

                        var wpmarker = new tileInfo(Context, WindowInfo, textureSemaphore);
                        wpmarker.idtexture = isSpecialLabel ? greenAlt : green;

                        double markerHalfSize = _waypointMarkerSize;
                        // Calculate angle from waypoint to camera so marker always faces camera
                        double dx = cameraX - co[0];
                        double dy = cameraY - co[1];
                        double angleToCamera = Math.Atan2(dx, dy);
                        double sinAngle = Math.Sin(angleToCamera + Math.PI / 2);
                        double cosAngle = Math.Cos(angleToCamera + Math.PI / 2);

                        // Rotation around the axis facing the camera (perpendicular to billboard)
                        double rotationAngle = (DateTime.Now.TimeOfDay.TotalSeconds * 30.0) % 360.0;
                        double rotRad = MathHelper.Radians(rotationAngle);
                        double cosRot = Math.Cos(rotRad);
                        double sinRot = Math.Sin(rotRad);

                        // Corner offsets in local 2D space (horizontal, vertical)
                        // tr: (+1, +1), tl: (-1, +1), br: (+1, -1), bl: (-1, -1)
                        double[][] corners = new double[][] {
                            new double[] { 1, 1, 1, 0 },   // tr + tex coords (flipped U)
                            new double[] { -1, 1, 0, 0 },  // tl
                            new double[] { 1, -1, 1, 1 },  // br
                            new double[] { -1, -1, 0, 1 }  // bl
                        };

                        foreach (var corner in corners)
                        {
                            // Rotate in local 2D space (horizontal = along billboard width, vertical = along Z)
                            double localH = corner[0] * cosRot - corner[1] * sinRot;
                            double localV = corner[0] * sinRot + corner[1] * cosRot;

                            // Transform to world coordinates
                            wpmarker.vertex.Add(new Vertex(
                                co[0] + sinAngle * localH * markerHalfSize,
                                co[1] + cosAngle * localH * markerHalfSize,
                                wpAlt + localV * markerHalfSize,
                                0, 0, 0, 1, corner[2], corner[3]));
                        }

                        var startindex = (uint)wpmarker.vertex.Count - 4;
                        wpmarker.indices.AddRange(new[]
                                        {
                                startindex + 1, startindex + 2, startindex + 0,
                                startindex + 1, startindex + 3, startindex + 2
                            });

                        // Disable depth test for Home marker so it renders fully even when underground
                        bool isHomeMarker = point.Tag == "H";
                        if (isHomeMarker)
                            GL.Disable(EnableCap.DepthTest);

                        wpmarker.Draw(pass2ProjMatrix, modelMatrix);
                        wpmarker.Cleanup(true);

                        // Draw waypoint label at top of sprite (no rotation)
                        if (wpLabel != null)
                        {
                            int wpNumberTex = getWpLabelTexture(wpLabel);
                            if (wpNumberTex != 0)
                            {
                                var wpnumber = new tileInfo(Context, WindowInfo, textureSemaphore);
                                wpnumber.idtexture = wpNumberTex;

                                // H, R, G labels are centered and shifted down into marker, numbers are at top
                                bool centerLabel = isSpecialLabel;
                                double numberHalfSize = centerLabel ? markerHalfSize * 0.6 : markerHalfSize * 0.4;
                                double numberOffsetZ = centerLabel ? -(markerHalfSize / 20f) : markerHalfSize * 0.5;

                                // Static corners (no rotation applied), shifted up for numbers
                                // Flip horizontally by negating corner[0] to unmirror the number
                                foreach (var corner in corners)
                                {
                                    wpnumber.vertex.Add(new Vertex(
                                        co[0] - sinAngle * corner[0] * numberHalfSize,
                                        co[1] - cosAngle * corner[0] * numberHalfSize,
                                        wpAlt + corner[1] * numberHalfSize + numberOffsetZ,
                                        0, 0, 0, 1, corner[2], corner[3]));
                                }

                                var numStartindex = (uint)wpnumber.vertex.Count - 4;
                                wpnumber.indices.AddRange(new[]
                                {
                                    numStartindex + 1, numStartindex + 2, numStartindex + 0,
                                    numStartindex + 1, numStartindex + 3, numStartindex + 2
                                });

                                wpnumber.Draw(pass2ProjMatrix, modelMatrix);
                                wpnumber.Cleanup(true);
                            }
                        }

                        // Re-enable depth test after Home marker and its label are drawn
                        if (isHomeMarker)
                            GL.Enable(EnableCap.DepthTest);
                    }

                    GL.Disable(EnableCap.Blend);
                    GL.DepthMask(true);
                }

                // Draw plane, heading lines, and trail (all part of Pass 2 with same projection)
                if (IsVehicleConnected)
                {
                    // Draw heading (red) and nav bearing (orange) lines from plane center (skip in FPV mode)
                    if (!_fpvMode)
                        DrawHeadingLines(pass2ProjMatrix, modelMatrix);

                    // Draw flight path trail
                    DrawTrail(pass2ProjMatrix, modelMatrix);

                    // Draw plane (skip in FPV mode)
                    if (!_fpvMode)
                    {
                        DrawPlane(pass2ProjMatrix, modelMatrix);
                    }
                }

                _fpsOverlay.UpdateAndDraw();
                var beforeswapbuffer = DateTime.Now;
                try
                {
                    this.SwapBuffers();
                }
                catch
                {
                }

                try
                {
                    if (Context.IsCurrent)
                        Context.MakeCurrent(null);
                }
                catch
                {
                }

                //this.Invalidate();
                var delta = DateTime.Now - start;
                //Console.Write("OpenGLTest2 {0}    \r", delta.TotalMilliseconds);
                if (delta.TotalMilliseconds > 20)
                    Console.Write("OpenGLTest2 total {0} swap {1} wps {2} draw {3} clear {4} wait {5} bwait {6} wpmark {7}  \n",
                        delta.TotalMilliseconds,
                        (beforeswapbuffer - start).TotalMilliseconds,
                        (beforewps - start).TotalMilliseconds,
                        (beforedraw - start).TotalMilliseconds,
                        (beforeclear - start).TotalMilliseconds,
                        (afterwait - start).TotalMilliseconds,
                        (beforewait - start).TotalMilliseconds,
                        (beforewpsmarkers - start).TotalMilliseconds);
            }
            finally
            {
                textureSemaphore.Release();
            }
        }

        private double[] projectLocation(double[] oldpos)
        {
            var newloc = LocationProjection.Project(_center, _velocity, _centerTime, DateTime.Now);
            var newpos = convertCoords(newloc);

            // Initialize Kalman filters on first run
            if (!_kalmanInitialized)
            {
                _kalmanPosX.Reset(newpos[0]);
                _kalmanPosY.Reset(newpos[1]);
                _kalmanPosZ.Reset(newpos[2]);
                _kalmanInitialized = true;
            }

            // Use Kalman filter for smooth interpolation
            return new double[]
            {
                _kalmanPosX.Update(newpos[0]),
                _kalmanPosY.Update(newpos[1]),
                _kalmanPosZ.Update(newpos[2])
            };
        }

        private bool _rotationKalmanInitialized = false;
        private MissionPlanner.Utilities.Vector3 filterRotation(MissionPlanner.Utilities.Vector3 rawRpy)
        {
            // Initialize rotation Kalman filters on first run to prevent slow rotation from 0
            if (!_rotationKalmanInitialized)
            {
                _kalmanRoll.Reset(rawRpy.X);
                _kalmanPitch.Reset(rawRpy.Y);
                _kalmanYaw.Reset(rawRpy.Z);
                _rotationKalmanInitialized = true;
            }

            return new MissionPlanner.Utilities.Vector3(
                (float)_kalmanRoll.UpdateAngle(rawRpy.X),
                (float)_kalmanPitch.UpdateAngle(rawRpy.Y),
                (float)_kalmanYaw.UpdateAngle(rawRpy.Z)
            );
        }

        private void ResetKalmanFilters()
        {
            _kalmanInitialized = false;
            _rotationKalmanInitialized = false;

            _kalmanPosX.Reset(0);
            _kalmanPosY.Reset(0);
            _kalmanPosZ.Reset(0);
            _kalmanRoll.Reset(0);
            _kalmanPitch.Reset(0);
            _kalmanYaw.Reset(0);
            _kalmanNavBearing.Reset(0);
            _kalmanGpsHeading.Reset(0);
        }

        private int Comparison(KeyValuePair<GPoint, tileInfo> x, KeyValuePair<GPoint, tileInfo> y)
        {
            return x.Value.zoom.CompareTo(y.Value.zoom);
        }

        private bool IsGuidedWaypoint(PointLatLngAlt point)
        {
            var guided = MainV2.comPort?.MAV?.GuidedMode;
            if (guided == null || guided.Value.x == 0 && guided.Value.y == 0)
                return false;

            double guidedLat = guided.Value.x / 1e7;
            double guidedLng = guided.Value.y / 1e7;

            const double tolerance = 0.0001;
            return Math.Abs(point.Lat - guidedLat) < tolerance &&
                   Math.Abs(point.Lng - guidedLng) < tolerance;
        }

        private void DrawSkyGradient()
        {
            // Draw a full-screen quad with gradient colors from ThemeManager
            // Top half: gradient from skyTop to skyBot
            // Bottom half: solid skyBot color
            var orthoProj = Matrix4.CreateOrthographicOffCenter(0, Width, 0, Height, -1, 1);
            var identity = Matrix4.Identity;

            // Get sky colors from theme (with fallback defaults if theme not yet loaded)
            Color skyTop = ThemeManager.HudSkyTop.A > 0 ? ThemeManager.HudSkyTop : Color.Blue;
            Color skyBot = ThemeManager.HudSkyBot.A > 0 ? ThemeManager.HudSkyBot : Color.LightBlue;

            // Lighten colors by 2x to match HUD appearance (blend towards white)
            float topR = Math.Min(1.0f, (skyTop.R / 255f) * 0.5f + 0.5f);
            float topG = Math.Min(1.0f, (skyTop.G / 255f) * 0.5f + 0.5f);
            float topB = Math.Min(1.0f, (skyTop.B / 255f) * 0.5f + 0.5f);
            float botR = Math.Min(1.0f, (skyBot.R / 255f) * 0.5f + 0.5f);
            float botG = Math.Min(1.0f, (skyBot.G / 255f) * 0.5f + 0.5f);
            float botB = Math.Min(1.0f, (skyBot.B / 255f) * 0.5f + 0.5f);

            float midY = Height / 2f;

            // Use Lines shader which supports vertex colors
            GL.UseProgram(Lines.Program);
            GL.UniformMatrix4(Lines.modelViewSlot, 1, false, ref identity.Row0.X);
            GL.UniformMatrix4(Lines.projectionSlot, 1, false, ref orthoProj.Row0.X);

            // Create vertices: bottom half solid + top half gradient
            var skyVerts = new List<Vertex>();
            // Bottom half - solid skyBot color (vertices 0-3)
            skyVerts.Add(new Vertex(0, 0, 0, botR, botG, botB, 1.0, 0, 0));      // Bottom-left
            skyVerts.Add(new Vertex(Width, 0, 0, botR, botG, botB, 1.0, 0, 0));  // Bottom-right
            skyVerts.Add(new Vertex(0, midY, 0, botR, botG, botB, 1.0, 0, 0));   // Mid-left
            skyVerts.Add(new Vertex(Width, midY, 0, botR, botG, botB, 1.0, 0, 0)); // Mid-right
            // Top half - gradient from skyBot to skyTop (vertices 4-5)
            skyVerts.Add(new Vertex(0, Height, 0, topR, topG, topB, 1.0, 0, 0)); // Top-left
            skyVerts.Add(new Vertex(Width, Height, 0, topR, topG, topB, 1.0, 0, 0)); // Top-right

            // Upload vertices
            int vbo;
            GL.GenBuffers(1, out vbo);
            GL.BindBuffer(BufferTarget.ArrayBuffer, vbo);
            var vertArray = skyVerts.ToArray();
            GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(vertArray.Length * Vertex.Stride), vertArray, BufferUsageHint.StreamDraw);

            // Set up vertex attributes
            GL.EnableVertexAttribArray(Lines.positionSlot);
            GL.VertexAttribPointer(Lines.positionSlot, 3, VertexAttribPointerType.Float, false, Vertex.Stride, 0);
            GL.EnableVertexAttribArray(Lines.colorSlot);
            GL.VertexAttribPointer(Lines.colorSlot, 4, VertexAttribPointerType.Float, false, Vertex.Stride, 12);

            // Draw as triangle strip: 0-1-2-3 (bottom half), then 2-3-4-5 (top half)
            GL.DrawArrays(PrimitiveType.TriangleStrip, 0, 6);

            // Cleanup
            GL.DisableVertexAttribArray(Lines.positionSlot);
            GL.DisableVertexAttribArray(Lines.colorSlot);
            GL.DeleteBuffer(vbo);
        }

        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.btn_configure = new MissionPlanner.Controls.MyButton();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.SuspendLayout();
            // 
            // btn_configure
            // 
            this.btn_configure.Location = new System.Drawing.Point(4, 4);
            this.btn_configure.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.btn_configure.Name = "btn_configure";
            this.btn_configure.Size = new System.Drawing.Size(70, 28);
            this.btn_configure.TabIndex = 0;
            this.btn_configure.Text = "Settings";
            this.btn_configure.TextColorNotEnabled = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(87)))), ((int)(((byte)(4)))));
            this.btn_configure.UseVisualStyleBackColor = true;
            this.btn_configure.Click += new System.EventHandler(this.btn_configure_Click);
            // 
            // timer1
            // 
            this.timer1.Interval = 12;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // Map3D
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.Controls.Add(this.btn_configure);
            this.Margin = new System.Windows.Forms.Padding(5, 5, 5, 5);
            this.Name = "Map3D";
            this.Size = new System.Drawing.Size(853, 591);
            this.Load += new System.EventHandler(this.test_Load);
            this.Resize += new System.EventHandler(this.test_Resize);
            this.ResumeLayout(false);

        }

        private void test_Load(object sender, EventArgs e)
        {
            _settingsLoaded = true;

            // Defer initialization until control is fully created
            BeginInvoke((Action)(() =>
            {
                if (IsDisposed || Disposing || Context == null)
                    return;

                try
                {
                    if (!Context.IsCurrent)
                        Context.MakeCurrent(this.WindowInfo);

                    _imageloaderThread = new Thread(imageLoader)
                    {
                        IsBackground = true,
                        Name = "gl imageLoader"
                    };
                    _imageloaderThread.Start();

                    // Request driver-side anti-aliasing (works when the context was created with samples).
                    GL.Enable(EnableCap.DepthTest);
                    GL.Enable(EnableCap.Lighting);
                    GL.Enable(EnableCap.Light0);
                    GL.Enable(EnableCap.ColorMaterial);
                    GL.Enable(EnableCap.Normalize);
                    GL.Enable(EnableCap.CullFace);
                    GL.Enable(EnableCap.Texture2D);
                    var preload = tileInfo.Program;
                    test_Resize(null, null);
                }
                catch (OpenTK.Graphics.GraphicsContextException)
                {
                    // Context failed to initialize, will retry on next paint
                }
            }));
        }

        private void btn_configure_Click(object sender, EventArgs e)
        {
            using (var dialog = new Form())
            {
                dialog.Text = "3D Map Settings";
                dialog.FormBorderStyle = FormBorderStyle.FixedDialog;
                dialog.StartPosition = FormStartPosition.CenterParent;
                dialog.MaximizeBox = false;
                dialog.MinimizeBox = false;
                dialog.AutoSize = true;
                dialog.AutoSizeMode = AutoSizeMode.GrowAndShrink;
                dialog.Padding = new Padding(10);

                // Main layout panel
                var mainLayout = new TableLayoutPanel
                {
                    AutoSize = true,
                    AutoSizeMode = AutoSizeMode.GrowAndShrink,
                    ColumnCount = 2,
                    Dock = DockStyle.Fill,
                    Padding = new Padding(5)
                };
                mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
                mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));

                int row = 0;
                int inputWidth = 100;

                // Helper to add a label + control row
                Action<string, Control> addRow = (labelText, control) =>
                {
                    var lbl = new Label { Text = labelText, AutoSize = true, Anchor = AnchorStyles.Left, Margin = new Padding(3, 6, 3, 3) };
                    control.Width = inputWidth;
                    control.Anchor = AnchorStyles.Left;
                    mainLayout.Controls.Add(lbl, 0, row);
                    mainLayout.Controls.Add(control, 1, row);
                    row++;
                };

                // Helper to add a full-width checkbox row
                Action<CheckBox> addCheckboxRow = (chk) =>
                {
                    chk.AutoSize = true;
                    chk.Anchor = AnchorStyles.Left;
                    chk.Margin = new Padding(3, 3, 3, 3);
                    mainLayout.SetColumnSpan(chk, 2);
                    mainLayout.Controls.Add(chk, 0, row);
                    row++;
                };

                // Numeric inputs
                var numDist = new NumericUpDown { Minimum = (decimal)0.1, Maximum = 100, DecimalPlaces = 2, Increment = (decimal)0.05, Value = (decimal)Math.Max(0.1, Math.Min(100, _cameraDist)) };
                addRow("Camera Dist:", numDist);

                var numHeight = new NumericUpDown { Minimum = -100, Maximum = 100, DecimalPlaces = 2, Increment = (decimal)0.05, Value = (decimal)Math.Max(-100, Math.Min(100, _cameraHeight)) };
                addRow("Camera Height:", numHeight);

                var numFOV = new NumericUpDown { Minimum = 30, Maximum = 120, Increment = 5, Value = (decimal)Math.Max(30, Math.Min(120, _cameraFOV)) };
                addRow("Camera FoV:", numFOV);

                var numScale = new NumericUpDown { Minimum = (decimal)0.1, Maximum = 10, DecimalPlaces = 2, Increment = (decimal)0.05, Value = (decimal)Math.Max(0.1, Math.Min(10, _planeScaleMultiplier)) };
                addRow("MAV Scale (m):", numScale);

                var numMarkerSize = new NumericUpDown { Minimum = 10, Maximum = 500, DecimalPlaces = 0, Increment = 10, Value = (decimal)Math.Max(10, Math.Min(500, _waypointMarkerSize)) };
                addRow("WP Marker Size:", numMarkerSize);

                var numADSBSize = new NumericUpDown { Minimum = 50, Maximum = 2000, DecimalPlaces = 0, Increment = 50, Value = (decimal)Math.Max(50, Math.Min(2000, _adsbCircleSize)) };
                addRow("ADSB Size:", numADSBSize);

                // Color picker
                Color selectedColor = _planeColor;
                var pnlColor = new Panel { Height = 23, BorderStyle = BorderStyle.FixedSingle, Cursor = Cursors.Hand, Tag = "IgnoreTheme", BackColor = _planeColor };
                pnlColor.Click += (s, ev) =>
                {
                    using (var colorDialog = new ColorDialog())
                    {
                        colorDialog.Color = selectedColor;
                        colorDialog.FullOpen = true;
                        if (colorDialog.ShowDialog() == DialogResult.OK)
                        {
                            selectedColor = colorDialog.Color;
                            pnlColor.BackColor = selectedColor;
                        }
                    }
                };
                addRow("MAV Color:", pnlColor);

                // STL file picker
                string selectedSTLPath = _stlLoader.CustomSTLPath;
                string stlButtonText = string.IsNullOrEmpty(selectedSTLPath) ? "Default" : Path.GetFileName(selectedSTLPath);
                var btnSTL = new MyButton { Text = stlButtonText, Height = 23 };
                btnSTL.Click += (s, ev) =>
                {
                    using (var openDialog = new OpenFileDialog())
                    {
                        openDialog.Filter = "STL Files (*.stl)|*.stl|All Files (*.*)|*.*";
                        openDialog.Title = "Select Plane STL File";
                        if (!string.IsNullOrEmpty(selectedSTLPath) && File.Exists(selectedSTLPath))
                            openDialog.InitialDirectory = Path.GetDirectoryName(selectedSTLPath);
                        if (openDialog.ShowDialog() == DialogResult.OK)
                        {
                            selectedSTLPath = openDialog.FileName;
                            btnSTL.Text = Path.GetFileName(selectedSTLPath);
                        }
                    }
                };
                addRow("STL File:", btnSTL);

                // Checkboxes
                var chkHeading = new CheckBox { Text = "Heading Line (Red)", Checked = _showHeadingLine };
                addCheckboxRow(chkHeading);

                var chkNavBearing = new CheckBox { Text = "Nav Bearing Line (Orange)", Checked = _showNavBearingLine };
                addCheckboxRow(chkNavBearing);

                var chkGpsHeading = new CheckBox { Text = "GPS Heading Line (Black)", Checked = _showGpsHeadingLine };
                addCheckboxRow(chkGpsHeading);

                var chkTurnRadius = new CheckBox { Text = "Turn Radius Arc (Pink)", Checked = _showTurnRadius };
                addCheckboxRow(chkTurnRadius);

                var chkTrail = new CheckBox { Text = "Flight Path Trail", Checked = _showTrail };
                addCheckboxRow(chkTrail);

                var chkFPV = new CheckBox { Text = "FPV Mode (camera at aircraft)", Checked = _fpvMode };
                chkFPV.CheckedChanged += (s, ev) =>
                {
                    numDist.Enabled = !chkFPV.Checked;
                    numHeight.Enabled = !chkFPV.Checked;
                };
                numDist.Enabled = !_fpvMode;
                numHeight.Enabled = !_fpvMode;
                addCheckboxRow(chkFPV);

                var chkDiskCache = new CheckBox { Text = "Disk Cache Tiles", Checked = _diskCacheTiles };
                addCheckboxRow(chkDiskCache);

                // Button panel - centered
                var buttonPanel = new FlowLayoutPanel
                {
                    AutoSize = true,
                    AutoSizeMode = AutoSizeMode.GrowAndShrink,
                    Anchor = AnchorStyles.None,
                    Margin = new Padding(0, 10, 0, 0)
                };

                var btnSave = new MyButton { Text = "Save", Width = 75, Margin = new Padding(10, 0, 0, 0) };
                btnSave.Click += (s, ev) => { dialog.Close(); };

                var btnReset = new MyButton { Text = "Reset", Width = 75, Margin = new Padding(10, 0, 0, 0) };
                btnReset.Click += (s, ev) =>
                {
                    numDist.Value = (decimal)0.8;
                    numHeight.Value = (decimal)0.2;
                    numFOV.Value = 60;
                    numScale.Value = 1;
                    numMarkerSize.Value = 60;
                    numADSBSize.Value = 500;
                    selectedColor = Color.Red;
                    pnlColor.BackColor = Color.Red;
                    selectedSTLPath = "";
                    btnSTL.Text = "Default";
                    chkHeading.Checked = true;
                    chkNavBearing.Checked = true;
                    chkGpsHeading.Checked = true;
                    chkTurnRadius.Checked = true;
                    chkTrail.Checked = false;
                    chkFPV.Checked = false;
                    chkDiskCache.Checked = true;
                    _cameraAngle = 0.0;
                    dialog.Close();
                };

                buttonPanel.Controls.Add(btnSave);
                buttonPanel.Controls.Add(btnReset);

                mainLayout.SetColumnSpan(buttonPanel, 2);
                mainLayout.Controls.Add(buttonPanel, 0, row);

                dialog.Controls.Add(mainLayout);

                ThemeManager.ApplyThemeTo(dialog);

                dialog.FormClosing += (s, ev) =>
                {
                    _cameraDist = (double)numDist.Value;
                    _cameraHeight = (double)numHeight.Value;
                    _planeScaleMultiplier = (float)numScale.Value;
                    _cameraFOV = (float)numFOV.Value;
                    _waypointMarkerSize = (double)numMarkerSize.Value;
                    _adsbCircleSize = (double)numADSBSize.Value;
                    _planeColor = selectedColor;
                    _showHeadingLine = chkHeading.Checked;
                    _showNavBearingLine = chkNavBearing.Checked;
                    _showGpsHeadingLine = chkGpsHeading.Checked;
                    _showTurnRadius = chkTurnRadius.Checked;
                    _showTrail = chkTrail.Checked;
                    _fpvMode = chkFPV.Checked;

                    // Handle disk cache setting - clear cache if disabled
                    bool oldDiskCache = _diskCacheTiles;
                    _diskCacheTiles = chkDiskCache.Checked;
                    if (oldDiskCache && !_diskCacheTiles)
                    {
                        try
                        {
                            Map3DTileCache.ClearCache();
                        }
                        catch (Exception ex)
                        {
                            Debug.WriteLine($"Failed to clear tile cache: {ex.Message}");
                        }
                    }

                    _stlLoader.CustomSTLPath = selectedSTLPath;

                    Settings.Instance["map3d_camera_dist"] = _cameraDist.ToString();
                    Settings.Instance["map3d_camera_height"] = _cameraHeight.ToString();
                    Settings.Instance["map3d_mav_scale"] = _planeScaleMultiplier.ToString();
                    Settings.Instance["map3d_fov"] = _cameraFOV.ToString();
                    Settings.Instance["map3d_waypoint_marker_size"] = _waypointMarkerSize.ToString();
                    Settings.Instance["map3d_adsb_size"] = _adsbCircleSize.ToString();
                    Settings.Instance["map3d_mav_color"] = _planeColor.ToArgb().ToString();
                    Settings.Instance["map3d_plane_stl_path"] = _stlLoader.CustomSTLPath;
                    Settings.Instance["map3d_show_heading"] = _showHeadingLine.ToString();
                    Settings.Instance["map3d_show_nav_bearing"] = _showNavBearingLine.ToString();
                    Settings.Instance["map3d_show_gps_heading"] = _showGpsHeadingLine.ToString();
                    Settings.Instance["map3d_show_turn_radius"] = _showTurnRadius.ToString();
                    Settings.Instance["map3d_show_trail"] = _showTrail.ToString();
                    Settings.Instance["map3d_fpv_mode"] = _fpvMode.ToString();
                    Settings.Instance["map3d_disk_cache_tiles"] = _diskCacheTiles.ToString();
                    Settings.Instance.Save();

                    test_Resize(null, null);
                };

                dialog.ShowDialog();
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (onpaintrun == true && IsHandleCreated && !IsDisposed && !Disposing)
            {
                this.Invalidate();
                onpaintrun = false;
            }
        }

        private void test_Resize(object sender, EventArgs e)
        {
            if (!IsHandleCreated || IsDisposed || Disposing || Context == null)
                return;

            textureSemaphore.Wait();
            try
            {
                if (!Context.IsCurrent)
                    Context.MakeCurrent(this.WindowInfo);

                GL.Viewport(0, 0, this.Width, this.Height);
                float renderDistance = _center.Alt > 500 ? 100000f : 50000f;
                projMatrix = OpenTK.Matrix4.CreatePerspectiveFieldOfView(
                    (float) (_cameraFOV * MathHelper.deg2rad),
                    (float) Width / Height, 0.1f,
                    renderDistance);
                GL.UniformMatrix4(tileInfo.projectionSlot, 1, false, ref projMatrix.Row0.X);
                {
                    // for unproject - updated on every draw
                    GL.GetInteger(GetPName.Viewport, viewport);
                }
                if (Context.IsCurrent)
                    Context.MakeCurrent(null);
            }
            finally
            {
                textureSemaphore.Release();
            }

            sizeChanged = true;
            this.Invalidate();
        }

        public class Lines: IDisposable
        {
            private static int _program;
            private static int VertexShader;
            private static int FragmentShader;
            internal static int positionSlot;
            internal static int colorSlot;
            internal static int projectionSlot;
            internal static int modelViewSlot;
            private static int textureSlot;
            private static int texCoordSlot;

            internal static int Program
            {
                get
                {
                    if (_program == 0)
                        CreateShaders();
                    return _program;
                }
            }

            private int ID_VBO;
            private int ID_EBO;
            private bool disposedValue;

            public List<Vertex> vertex { get; } = new List<Vertex>();
            public List<uint> indices { get; } = new List<uint>();
            public float Width { get; set; } = 1.0f;

            public void Add(double x,double y, double z, double r, double g, double b, double a)
            {
                vertex.Add(new Vertex(x, y, z, r, g, b, a, 0, 0));
                indices.Add((uint)(vertex.Count - 1));
            }

            /// <summary>
            /// Vertex Buffer
            /// </summary>
            public int idVBO
            {
                get
                {
                    if (ID_VBO != 0)
                        return ID_VBO;
                    GL.GenBuffers(1, out ID_VBO);
                    if (ID_VBO == 0)
                        return ID_VBO;
                    GL.BindBuffer(BufferTarget.ArrayBuffer, ID_VBO);
                    GL.BufferData(BufferTarget.ArrayBuffer, (vertex.Count * Vertex.Stride), vertex.ToArray(),
                        BufferUsageHint.DynamicDraw);
                    int bufferSize;
                    // Validate that the buffer is the correct size
                    GL.GetBufferParameter(BufferTarget.ArrayBuffer, BufferParameterName.BufferSize, out bufferSize);
                    if (vertex.Count * Vertex.Stride != bufferSize)
                        throw new ApplicationException("Vertex array not uploaded correctly");
                    GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
                    return ID_VBO;
                }
            }

            /// <summary>
            /// Element index Buffer
            /// </summary>
            public int idEBO
            {
                get
                {
                    if (ID_EBO != 0)
                        return ID_EBO;
                    GL.GenBuffers(1, out ID_EBO);
                    if (ID_EBO == 0)
                        return ID_EBO;
                    GL.BindBuffer(BufferTarget.ElementArrayBuffer, ID_EBO);
                    GL.BufferData(BufferTarget.ElementArrayBuffer, (indices.Count * sizeof(uint)), indices.ToArray(),
                        BufferUsageHint.DynamicDraw);
                    int bufferSize;
                    // Validate that the buffer is the correct size
                    GL.GetBufferParameter(BufferTarget.ElementArrayBuffer, BufferParameterName.BufferSize,
                        out bufferSize);
                    if (indices.Count * sizeof(int) != bufferSize)
                        throw new ApplicationException("Element array not uploaded correctly");
                    GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);
                    return ID_EBO;
                }
            }

            public void Draw(Matrix4 Projection, Matrix4 ModelView)
            {
                if (idVBO == 0 || idEBO == 0)
                    return;

                if (_program == 0)
                    CreateShaders();

                // use the shader
                GL.UseProgram(_program);

                // enable position
                GL.EnableVertexAttribArray(positionSlot);
                // enable color
                GL.EnableVertexAttribArray(colorSlot);

                // set matrix
                GL.UniformMatrix4(modelViewSlot, 1, false, ref ModelView.Row0.X);
                GL.UniformMatrix4(projectionSlot, 1, false, ref Projection.Row0.X);

                // set linewidth in px
                GL.LineWidth(Width);

                // bind the vertex buffers
                GL.BindBuffer(BufferTarget.ArrayBuffer, idVBO);
                GL.BindBuffer(BufferTarget.ElementArrayBuffer, idEBO);

                // map the vertex buffers
                GL.VertexAttribPointer(positionSlot, 3, VertexAttribPointerType.Float, false, Marshal.SizeOf(typeof(Vertex)), (IntPtr)0);
                GL.VertexAttribPointer(colorSlot, 4, VertexAttribPointerType.Float, false, Marshal.SizeOf(typeof(Vertex)), (IntPtr)(sizeof(float) * 3));
                GL.VertexAttribPointer(texCoordSlot, 2, VertexAttribPointerType.Float, false, Marshal.SizeOf(typeof(Vertex)), (IntPtr)(sizeof(float) * 7));

                // draw it
                GL.DrawArrays(PrimitiveType.LineStrip, 0, indices.Count);

                // disable vertex array
                GL.DisableVertexAttribArray(positionSlot);
                GL.DisableVertexAttribArray(colorSlot);
            }

            private static void CreateShaders()
            {
                VertexShader = GL.CreateShader(ShaderType.VertexShader);
                FragmentShader = GL.CreateShader(ShaderType.FragmentShader);

                //https://webglfundamentals.org/webgl/lessons/webgl-fog.html
                //http://www.ozone3d.net/tutorials/glsl_fog/p04.php

                GL.ShaderSource(VertexShader, @"
attribute vec3 Position;
attribute vec4 SourceColor;
attribute vec2 TexCoordIn;
varying vec4 DestinationColor;
varying vec2 TexCoordOut;
uniform mat4 Projection;
uniform mat4 ModelView;
void main(void) {
    DestinationColor = SourceColor;
    gl_Position = Projection * ModelView * vec4(Position, 1.0);
    TexCoordOut = TexCoordIn;
}
                ");
                GL.ShaderSource(FragmentShader, @"
varying vec4 DestinationColor;
varying vec2 TexCoordOut;
uniform sampler2D Texture;
void main(void) {
    float z = gl_FragCoord.z / gl_FragCoord.w;
    gl_FragColor = DestinationColor;
}
                ");
                GL.CompileShader(VertexShader);
                Debug.WriteLine(GL.GetShaderInfoLog(VertexShader));
                GL.GetShader(VertexShader, ShaderParameter.CompileStatus, out var code);
                if (code != (int)All.True)
                {
                    // We can use `GL.GetShaderInfoLog(shader)` to get information about the error.
                    throw new Exception(
                        $"Error occurred whilst compiling Shader({VertexShader}) {GL.GetShaderInfoLog(VertexShader)}");
                }

                GL.CompileShader(FragmentShader);
                Debug.WriteLine(GL.GetShaderInfoLog(FragmentShader));
                GL.GetShader(FragmentShader, ShaderParameter.CompileStatus, out code);
                if (code != (int)All.True)
                {
                    // We can use `GL.GetShaderInfoLog(shader)` to get information about the error.
                    throw new Exception(
                        $"Error occurred whilst compiling Shader({FragmentShader}) {GL.GetShaderInfoLog(FragmentShader)}");
                }

                _program = GL.CreateProgram();
                GL.AttachShader(_program, VertexShader);
                GL.AttachShader(_program, FragmentShader);
                GL.LinkProgram(_program);
                Debug.WriteLine(GL.GetProgramInfoLog(_program));
                GL.GetProgram(_program, GetProgramParameterName.LinkStatus, out code);
                if (code != (int)All.True)
                {
                    // We can use `GL.GetProgramInfoLog(program)` to get information about the error.
                    throw new Exception(
                        $"Error occurred whilst linking Program({_program}) {GL.GetProgramInfoLog(_program)}");
                }

                GL.UseProgram(_program);
                positionSlot = GL.GetAttribLocation(_program, "Position");
                colorSlot = GL.GetAttribLocation(_program, "SourceColor");
                texCoordSlot = GL.GetAttribLocation(_program, "TexCoordIn");
                projectionSlot = GL.GetUniformLocation(_program, "Projection");
                modelViewSlot = GL.GetUniformLocation(_program, "ModelView");
                textureSlot = GL.GetUniformLocation(_program, "Texture");
            }

            protected virtual void Dispose(bool disposing)
            {
                if (!disposedValue)
                {
                    if (disposing)
                    {
                        // TODO: dispose managed state (managed objects)
                    }

                    GL.DeleteBuffers(1, ref ID_VBO);
                    GL.DeleteBuffers(1, ref ID_EBO);
                    disposedValue = true;
                }
            }

            public void Dispose()
            {
                // Do not change this code. Put cleanup code in 'Dispose(bool disposing)' method
                Dispose(disposing: true);
                GC.SuppressFinalize(this);
            }
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct Vertex
        {
            //https://learnopengl.com/Getting-started/Textures
            public float X;
            public float Y;
            public float Z;
            public float R;
            public float G;
            public float B;
            public float A;
            public float S;
            public float T;
            /// <summary>
            /// Vertex
            /// </summary>
            /// <param name="x">position</param>
            /// <param name="y">position</param>
            /// <param name="z">position</param>
            /// <param name="r">color</param>
            /// <param name="g">color</param>
            /// <param name="b">color</param>
            /// <param name="a">color</param>
            /// <param name="s">texture</param>
            /// <param name="t">texture</param>
            public Vertex(double x, double y, double z, double r, double g, double b, double a, double s, double t)
            {
                X = (float)x;
                Y = (float)y;
                Z = (float)z;
                R = (float)r;
                G = (float)g;
                B = (float)b;
                A = (float)a;
                S = (float)s;
                T = (float)t;
                if (S > 1 || S < 0 || T > 1 || T < 0)
                {
                }
            }

            public static readonly int Stride = System.Runtime.InteropServices.Marshal.SizeOf(new Vertex());
        }

        /// <summary>
        /// Stores ADSB aircraft screen position for hit testing
        /// </summary>
        private struct ADSBScreenPosition
        {
            public float ScreenX;
            public float ScreenY;
            public float Radius; // Screen radius for hit testing
            public adsb.PointLatLngAltHdg PlaneData;
            public double DistanceToOwn; // Distance in meters to own aircraft
        }

        private int ComputeWaypointHash(IList<PointLatLngAlt> waypoints)
        {
            unchecked
            {
                int hash = 17;
                for (int i = 0; i < waypoints.Count; i++)
                {
                    var wp = waypoints[i];
                    if (wp == null)
                        continue;
                    hash = hash * 31 + wp.Lat.GetHashCode();
                    hash = hash * 31 + wp.Lng.GetHashCode();
                    hash = hash * 31 + wp.Alt.GetHashCode();
                    hash = hash * 31 + (wp.Tag?.GetHashCode() ?? 0);
                }
                return hash;
            }
        }

        private List<PointLatLngAlt> _cachedWaypointList = new List<PointLatLngAlt>();
        private int _cachedWpsHash = 0;

        private List<PointLatLngAlt> GetWaypointListFromMAV()
        {
            try
            {
                var wps = MainV2.comPort.MAV.wps;
                if (wps == null || wps.Count == 0)
                {
                    _cachedWaypointList.Clear();
                    _cachedWpsHash = 0;
                    return _cachedWaypointList;
                }

                // Compute hash to check if wps changed
                int newHash;
                unchecked
                {
                    newHash = 17;
                    foreach (var kvp in wps)
                    {
                        var wp = kvp.Value;
                        newHash = newHash * 31 + wp.seq.GetHashCode();
                        newHash = newHash * 31 + wp.x.GetHashCode();
                        newHash = newHash * 31 + wp.y.GetHashCode();
                        newHash = newHash * 31 + wp.z.GetHashCode();
                        newHash = newHash * 31 + wp.command.GetHashCode();
                    }
                }

                if (newHash == _cachedWpsHash && _cachedWaypointList.Count > 0)
                    return _cachedWaypointList;

                _cachedWpsHash = newHash;
                _cachedWaypointList.Clear();

                // Get home location
                var home = PointLatLngAlt.Zero;
                if (MainV2.comPort.MAV.cs.HomeLocation.Lat != 0 || MainV2.comPort.MAV.cs.HomeLocation.Lng != 0)
                {
                    home = new PointLatLngAlt(MainV2.comPort.MAV.cs.HomeLocation.Lat,
                        MainV2.comPort.MAV.cs.HomeLocation.Lng,
                        MainV2.comPort.MAV.cs.HomeLocation.Alt / CurrentState.multiplieralt, "H");
                }
                else if (MainV2.comPort.MAV.cs.PlannedHomeLocation.Lat != 0 || MainV2.comPort.MAV.cs.PlannedHomeLocation.Lng != 0)
                {
                    home = new PointLatLngAlt(MainV2.comPort.MAV.cs.PlannedHomeLocation.Lat,
                        MainV2.comPort.MAV.cs.PlannedHomeLocation.Lng,
                        MainV2.comPort.MAV.cs.PlannedHomeLocation.Alt / CurrentState.multiplieralt, "H");
                }

                if (home != PointLatLngAlt.Zero)
                    _cachedWaypointList.Add(home);

                // Convert mission items to PointLatLngAlt, skipping home (index 0)
                var wpsList = wps.Values.OrderBy(a => a.seq).ToList();
                foreach (var wp in wpsList)
                {
                    if (wp.seq == 0) continue; // Skip home

                    var cmd = wp.command;
                    // Only include navigatable waypoints (similar to WPOverlay logic)
                    if (cmd < (ushort)MAVLink.MAV_CMD.LAST &&
                        cmd != (ushort)MAVLink.MAV_CMD.RETURN_TO_LAUNCH &&
                        cmd != (ushort)MAVLink.MAV_CMD.CONTINUE_AND_CHANGE_ALT &&
                        cmd != (ushort)MAVLink.MAV_CMD.DELAY &&
                        cmd != (ushort)MAVLink.MAV_CMD.GUIDED_ENABLE
                        || cmd == (ushort)MAVLink.MAV_CMD.DO_SET_ROI || cmd == (ushort)MAVLink.MAV_CMD.DO_LAND_START)
                    {
                        double lat = wp.x / 1e7;
                        double lng = wp.y / 1e7;
                        double alt = wp.z;

                        // Skip 0,0 locations for land commands
                        if ((cmd == (ushort)MAVLink.MAV_CMD.LAND || cmd == (ushort)MAVLink.MAV_CMD.VTOL_LAND) && lat == 0 && lng == 0)
                            continue;

                        if (lat == 0 && lng == 0)
                        {
                            _cachedWaypointList.Add(null);
                            continue;
                        }

                        var point = new PointLatLngAlt(lat, lng, alt, wp.seq.ToString());
                        if (cmd == (ushort)MAVLink.MAV_CMD.DO_SET_ROI)
                            point.Tag = "ROI" + wp.seq;
                        _cachedWaypointList.Add(point);
                    }
                    else
                    {
                        _cachedWaypointList.Add(null);
                    }
                }

                return _cachedWaypointList;
            }
            catch
            {
                return _cachedWaypointList;
            }
        }

        private class FpsOverlay
        {
            private readonly Stopwatch _watch = new Stopwatch();
            private int _frameCount = 0;
            private double _fpsValue = 0;
            private int _textureId = 0;
            private int _texWidth = 0;
            private int _texHeight = 0;
            private bool _dirty = true;
            private const int Padding = 8;
            private static int _program = 0;
            private static int _posSlot = 0;
            private static int _texSlot = 0;
            private static int _samplerSlot = 0;

            public FpsOverlay()
            {
                _watch.Start();
            }

            public void UpdateAndDraw()
            {
                var control = Map3D.instance;
                if (control == null)
                    return;

                _frameCount++;
                if (_watch.ElapsedMilliseconds >= 1000)
                {
                    _fpsValue = _frameCount / (_watch.ElapsedMilliseconds / 1000.0);
                    _frameCount = 0;
                    _watch.Restart();
                    _dirty = true;
                }

                DrawOverlay(control);
            }

            private void DrawOverlay(Map3D control)
            {
                if (_textureId == 0 || _dirty)
                {
                    UpdateTexture();
                }

                if (_textureId == 0 || control.Width == 0 || control.Height == 0)
                    return;

                EnsureProgram();

                // Convert pixel coords to NDC [-1, 1], origin top-left in pixels.
                float left = -1f + (2f * Padding / control.Width);
                float right = -1f + (2f * (Padding + _texWidth) / control.Width);
                float top = 1f - (2f * (control.Height - Padding - _texHeight) / control.Height);
                float bottom = 1f - (2f * (control.Height - Padding) / control.Height);

                float[] quad =
                {
                    left,  bottom, 0f, 1f,
                    right, bottom, 1f, 1f,
                    left,  top,    0f, 0f,
                    right, top,    1f, 0f
                };

                int vbo = 0;
                GL.GenBuffers(1, out vbo);
                GL.BindBuffer(BufferTarget.ArrayBuffer, vbo);
                GL.BufferData(BufferTarget.ArrayBuffer, quad.Length * sizeof(float), quad, BufferUsageHint.DynamicDraw);

                GL.UseProgram(_program);
                GL.Enable(EnableCap.Blend);
                GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
                GL.Disable(EnableCap.DepthTest);

                GL.ActiveTexture(TextureUnit.Texture0);
                GL.BindTexture(TextureTarget.Texture2D, _textureId);
                GL.Uniform1(_samplerSlot, 0);

                int stride = 4 * sizeof(float);
                GL.VertexAttribPointer(_posSlot, 2, VertexAttribPointerType.Float, false, stride, IntPtr.Zero);
                GL.EnableVertexAttribArray(_posSlot);
                GL.VertexAttribPointer(_texSlot, 2, VertexAttribPointerType.Float, false, stride, (IntPtr)(2 * sizeof(float)));
                GL.EnableVertexAttribArray(_texSlot);

                GL.DrawArrays(PrimitiveType.TriangleStrip, 0, 4);

                GL.DisableVertexAttribArray(_posSlot);
                GL.DisableVertexAttribArray(_texSlot);

                GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
                GL.DeleteBuffers(1, ref vbo);
            }

            private void UpdateTexture()
            {
                string text = $"FPS: {_fpsValue:F1}";

                using (var dummy = new Bitmap(1, 1))
                using (var g = Graphics.FromImage(dummy))
                using (var font = new Font("Segoe UI", 10, FontStyle.Bold, GraphicsUnit.Point))
                {
                    var size = g.MeasureString(text, font);
                    int width = (int)Math.Ceiling(size.Width + 8);
                    int height = (int)Math.Ceiling(size.Height + 4);

                    using (var bmp = new Bitmap(width, height, System.Drawing.Imaging.PixelFormat.Format32bppArgb))
                    using (var gbmp = Graphics.FromImage(bmp))
                    {
                        gbmp.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
                        gbmp.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;
                        gbmp.Clear(Color.Transparent);
                        using (var bgBrush = new SolidBrush(Color.FromArgb(160, 0, 0, 0)))
                        {
                            gbmp.FillRectangle(bgBrush, 0, 0, width, height);
                        }
                        using (var fgBrush = new SolidBrush(Color.White))
                        {
                            gbmp.DrawString(text, font, fgBrush, new PointF(4, 2));
                        }

                        var data = bmp.LockBits(new Rectangle(0, 0, bmp.Width, bmp.Height),
                            ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

                        if (_textureId != 0)
                        {
                            GL.DeleteTextures(1, ref _textureId);
                            _textureId = 0;
                        }

                        _textureId = CreateTexture(data);

                        // Ensure linear filtering for readability
                        GL.BindTexture(TextureTarget.Texture2D, _textureId);
                        GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Linear);
                        GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);

                        bmp.UnlockBits(data);

                        _texWidth = bmp.Width;
                        _texHeight = bmp.Height;
                    }
                }

                _dirty = false;
            }

            private void EnsureProgram()
            {
                if (_program != 0)
                    return;

                int vShader = GL.CreateShader(ShaderType.VertexShader);
                GL.ShaderSource(vShader, @"
attribute vec2 Position;
attribute vec2 TexCoordIn;
varying vec2 TexCoord;
void main(void) {
    gl_Position = vec4(Position, 0.0, 1.0);
    TexCoord = TexCoordIn;
}");
                GL.CompileShader(vShader);
                GL.GetShader(vShader, ShaderParameter.CompileStatus, out var code);
                if (code != (int)All.True)
                {
                    throw new Exception($"Overlay vertex shader compile error: {GL.GetShaderInfoLog(vShader)}");
                }

                int fShader = GL.CreateShader(ShaderType.FragmentShader);
                GL.ShaderSource(fShader, @"
precision mediump float;
varying vec2 TexCoord;
uniform sampler2D Texture;
void main(void) {
    gl_FragColor = texture2D(Texture, TexCoord);
}");
                GL.CompileShader(fShader);
                GL.GetShader(fShader, ShaderParameter.CompileStatus, out code);
                if (code != (int)All.True)
                {
                    throw new Exception($"Overlay fragment shader compile error: {GL.GetShaderInfoLog(fShader)}");
                }

                _program = GL.CreateProgram();
                GL.AttachShader(_program, vShader);
                GL.AttachShader(_program, fShader);
                GL.LinkProgram(_program);
                GL.GetProgram(_program, GetProgramParameterName.LinkStatus, out code);
                if (code != (int)All.True)
                {
                    throw new Exception($"Overlay program link error: {GL.GetProgramInfoLog(_program)}");
                }

                _posSlot = GL.GetAttribLocation(_program, "Position");
                _texSlot = GL.GetAttribLocation(_program, "TexCoordIn");
                _samplerSlot = GL.GetUniformLocation(_program, "Texture");
            }
        }
    }
}
