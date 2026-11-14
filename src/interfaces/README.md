# Typed contracts by responsibility

| Package | Contract |
|---|---|
| `control_interfaces` | Requested/effective control mode, permission and inhibit reasons |
| `mission_interfaces` | Task catalog, execution IDs, status and task services/actions |
| `health_interfaces` | Health signals and aggregated vessel state |
| `perception_interfaces` | Semantic buoy classes, ROIs and detections |
| `runtime_interfaces` | Runtime profile configuration and Nav2 readiness |
| `operator_interfaces` | Correlated operator requests/responses across the critical link |

Use ROS standard messages for ordinary data: `sensor_msgs/Joy` for RC/joystick,
`geometry_msgs/Twist` for velocity, `nav_msgs/Odometry` for motion feedback,
`diagnostic_msgs/DiagnosticArray` for diagnostic observations, and the existing
standard numeric messages at the microcontroller boundary. Custom messages here
preserve state enums, request/execution IDs and reasons that standard messages
do not express. Implementation packages depend only on the contracts they use.

The former `njord_interfaces` aggregate is retired. Update and rebuild both vessel
and ground consumers together. Historical bags retain their original type names;
use the recording revision to replay them. Field ordering and enum values of all
18 migrated definitions are preserved.
