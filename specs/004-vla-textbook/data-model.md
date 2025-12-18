---
description: "Data models and structures for Module 4: Vision-Language-Action (VLA) - Technical data specifications"
---

# Module 4: Vision-Language-Action (VLA) - Data Models

## Overview

This document defines the data models, structures, and formats used in the Vision-Language-Action module covering VLA models, speech recognition, and cognitive planning for humanoid robotics.

## Speech Recognition Data Models

### Audio Processing Data Model

```
AudioData {
  raw_audio: bytes[]           # Raw audio samples
  sample_rate: int32          # Sampling rate in Hz (e.g., 16000, 44100)
  channels: int32             # Number of audio channels (1 for mono, 2 for stereo)
  duration: float64           # Duration in seconds
  processed_audio: float32[]  # Processed audio samples (normalized, filtered)
  format: string              # Audio format (e.g., "WAV", "MP3", "FLAC")
  timestamp: builtin_interfaces/Time  # When audio was captured
}
```

### Speech Recognition Result Model

```
SpeechRecognitionResult {
  text: string                # Transcribed text
  confidence: float64         # Confidence score (0.0 to 1.0)
  segments: SpeechSegment[]   # Time-aligned segments
  language: string            # Detected language code (e.g., "en", "es", "fr")
  duration: float64           # Processing duration in seconds
  timestamp: builtin_interfaces/Time  # When recognition completed
}

SpeechSegment {
  text: string                # Text for this segment
  start_time: float64         # Start time in seconds from audio start
  end_time: float64           # End time in seconds from audio start
  confidence: float64         # Confidence for this segment
  tokens: int32[]             # Token IDs from tokenizer
}
```

## Natural Language Processing Data Models

### Command Parsing Data Model

```
NaturalLanguageCommand {
  raw_text: string                    # Original command text
  parsed_intent: Intent               # Parsed intent and parameters
  entities: Entity[]                  # Extracted entities
  action_sequence: ActionSequence     # Planned action sequence
  confidence: float64                 # Overall parsing confidence
  context: CommandContext             # Contextual information
  timestamp: builtin_interfaces/Time  # When command was processed
}

Intent {
  type: string                       # Intent type (e.g., "navigation", "manipulation", "greeting")
  parameters: map<string, string>    # Intent-specific parameters
  confidence: float64                # Intent classification confidence
}

Entity {
  text: string                       # Entity text
  type: string                       # Entity type (e.g., "location", "object", "person")
  start_pos: int32                   # Start position in original text
  end_pos: int32                     # End position in original text
  confidence: float64                # Entity recognition confidence
  resolved_value: string             # Resolved value (e.g., coordinates for location)
}
```

### Action Sequence Data Model

```
ActionSequence {
  id: string                         # Unique sequence identifier
  actions: RobotAction[]             # Ordered sequence of actions
  dependencies: ActionDependency[]   # Dependencies between actions
  estimated_duration: float64        # Estimated total execution time
  success_criteria: SuccessCriteria  # Criteria for sequence success
  fallback_actions: RobotAction[]    # Fallback actions if main sequence fails
}

RobotAction {
  id: string                         # Unique action identifier
  type: string                       # Action type (e.g., "move_to", "pick_object", "speak")
  parameters: map<string, string>    # Action-specific parameters
  required_resources: string[]       # Resources required for action
  timeout: float64                   # Maximum time allowed for execution
  success_conditions: Condition[]    # Conditions for successful completion
  error_handling: ErrorHandling      # How to handle errors
}

ActionDependency {
  action_id: string                  # ID of action that depends
  depends_on: string                 # ID of action it depends on
  dependency_type: string            # Type ("before", "after", "parallel")
}

Condition {
  type: string                       # Condition type (e.g., "position", "gripper_state")
  parameter: string                  # Parameter to check
  expected_value: string             # Expected value
  tolerance: float64                 # Tolerance for numerical comparisons
}

SuccessCriteria {
  all_actions_completed: bool        # Whether all actions must complete
  subset_required: string[]          # Subset of actions that must succeed
  quality_threshold: float64         # Minimum quality threshold
}
```

## Vision Processing Data Models

### Visual Perception Data Model

```
VisualPerceptionResult {
  objects: DetectedObject[]          # Detected objects in the scene
  spatial_relationships: SpatialRelationship[]  # Relationships between objects
  scene_description: string          # Natural language description of scene
  confidence: float64               # Overall confidence in perception
  timestamp: builtin_interfaces/Time # When perception was completed
}

DetectedObject {
  id: string                        # Unique object identifier
  class_name: string                # Object class (e.g., "chair", "cup", "person")
  bounding_box: BoundingBox         # 2D bounding box in image coordinates
  mask: PixelMask                   # Segmentation mask
  pose: geometry_msgs/Pose          # 3D pose in robot coordinate frame
  confidence: float64               # Detection confidence
  attributes: map<string, string>   # Object attributes (e.g., "color", "size")
}

BoundingBox {
  x: float64                        # X coordinate of top-left corner
  y: float64                        # Y coordinate of top-left corner
  width: float64                    # Width of bounding box
  height: float64                   # Height of bounding box
}

SpatialRelationship {
  subject_id: string                # ID of subject object
  predicate: string                 # Relationship type (e.g., "left_of", "on_top_of", "near")
  object_id: string                 # ID of object in relationship
  confidence: float64               # Relationship confidence
  distance: float64                 # Distance between objects (if applicable)
}
```

## Cognitive Planning Data Models

### Task Planning Data Model

```
TaskPlan {
  id: string                        # Unique plan identifier
  original_command: string          # Original natural language command
  high_level_tasks: HighLevelTask[] # Decomposed high-level tasks
  execution_context: ExecutionContext # Context for execution
  monitoring_plan: MonitoringPlan   # Plan for monitoring execution
  recovery_strategies: RecoveryStrategy[] # Strategies for handling failures
}

HighLevelTask {
  id: string                        # Unique task identifier
  description: string               # Natural language description of task
  subtasks: LowLevelAction[]        # Sequence of low-level actions
  preconditions: Condition[]        # Conditions that must be true before task
  postconditions: Condition[]       # Expected conditions after task completion
  success_criteria: SuccessCriteria # Criteria for task success
  estimated_time: float64          # Estimated time to complete
}

LowLevelAction {
  action_type: string               # Specific action type (e.g., "move_arm_to", "open_gripper")
  parameters: map<string, object>   # Action parameters
  ros_action: ROSActionSpec         # ROS 2 action specification
  timeout: float64                  # Maximum execution time
  retry_policy: RetryPolicy         # Policy for retries on failure
}

ROSActionSpec {
  action_name: string               # ROS 2 action name (e.g., "move_base_msgs/MoveBase")
  action_server: string             # Action server name
  goal_message: string              # Goal message content (JSON format)
  feedback_callback: string         # Callback for feedback
  result_callback: string           # Callback for result
}

RetryPolicy {
  max_attempts: int32               # Maximum number of retry attempts
  backoff_factor: float64           # Factor for exponential backoff
  timeout_factor: float64           # Factor to increase timeout with each attempt
}
```

### Monitoring and Execution Data Model

```
ExecutionState {
  current_action: string            # Currently executing action
  action_status: string             # Status (e.g., "pending", "executing", "completed", "failed")
  progress: float64                 # Progress percentage (0.0 to 1.0)
  start_time: builtin_interfaces/Time # When action started
  estimated_completion: builtin_interfaces/Time # Estimated completion time
  feedback_data: map<string, object> # Feedback from action execution
  error_info: ErrorInfo             # Error information if action failed
}

ErrorInfo {
  error_code: int32                 # Error code
  error_message: string             # Human-readable error message
  error_type: string                # Type of error (e.g., "timeout", "collision", "hardware")
  timestamp: builtin_interfaces/Time # When error occurred
  recovery_suggestions: string[]    # Suggested recovery actions
}

MonitoringPlan {
  conditions_to_monitor: Condition[] # Conditions to continuously monitor
  monitoring_frequency: float64     # Frequency of monitoring (Hz)
  alert_thresholds: AlertThreshold[] # Thresholds for alerts
  safety_constraints: SafetyConstraint[] # Safety constraints to enforce
}

AlertThreshold {
  condition: Condition              # Condition that triggers alert
  severity: string                  # Severity level (e.g., "info", "warning", "critical")
  notification_method: string[]     # How to notify (e.g., "log", "audio", "visual")
}
```

## LLM Integration Data Models

### LLM Request/Response Model

```
LLMRequest {
  prompt: string                    # Input prompt for the LLM
  model: string                     # LLM model identifier
  parameters: LLMParameters         # Model-specific parameters
  context: ConversationContext      # Conversation context
  timestamp: builtin_interfaces/Time # When request was made
}

LLMParameters {
  temperature: float64              # Sampling temperature (0.0 to 2.0)
  max_tokens: int32                 # Maximum tokens to generate
  top_p: float64                    # Top-p sampling parameter
  frequency_penalty: float64        # Frequency penalty (0.0 to 2.0)
  presence_penalty: float64         # Presence penalty (0.0 to 2.0)
  stop_sequences: string[]          # Sequences to stop generation
}

LLMResponse {
  generated_text: string            # Generated text from LLM
  tokens_used: int32                # Number of tokens used
  finish_reason: string             # Reason for finishing (e.g., "stop", "length", "content_filter")
  confidence: float64               # Confidence in the response
  parsed_actions: RobotAction[]     # Parsed actions from response
  metadata: map<string, object>     # Additional metadata
  timestamp: builtin_interfaces/Time # When response was received
}

ConversationContext {
  history: Message[]                # Conversation history
  current_topic: string             # Current conversation topic
  user_profile: UserProfile         # Information about the user
  environment_context: EnvironmentContext # Context about the environment
}

Message {
  role: string                      # Role (e.g., "user", "assistant", "system")
  content: string                   # Message content
  timestamp: builtin_interfaces/Time # When message was created
}
```

## ROS 2 Integration Data Models

### VLA-Specific ROS Messages

```protobuf
# In a custom message package (e.g., vla_msgs)

# Voice command message
message VoiceCommand {
  std_msgs/Header header
  string command_text          # The recognized voice command
  float64 confidence          # Confidence in the recognition
  string language_code        # Language of the command
  builtin_interfaces/Time timestamp  # When command was received
}

# Action plan message
message ActionPlan {
  std_msgs/Header header
  string plan_id              # Unique identifier for the plan
  RobotAction[] actions       # Sequence of actions
  string original_command     # Original natural language command
  float64 estimated_duration  # Estimated time to complete plan
  PlanStatus status          # Current status of the plan
}

# Robot action message
message RobotAction {
  string action_type          # Type of action (e.g., "navigation", "manipulation")
  map<string, string> parameters # Action-specific parameters
  float64 timeout            # Maximum time for action completion
  string[] dependencies      # IDs of actions this action depends on
  repeated string feedback_topics # Topics to monitor for feedback
}

# Plan status enum
enum PlanStatus {
  PENDING = 0
  RUNNING = 1
  COMPLETED = 2
  FAILED = 3
  CANCELLED = 4
}
```

### Audio Processing ROS Messages

```protobuf
# Audio-related messages for VLA systems

# Audio data message
message AudioData {
  std_msgs/Header header
  uint8[] audio_data          # Raw audio samples
  uint32 sample_rate         # Sampling rate in Hz
  uint32 channels            # Number of channels
  string encoding            # Audio encoding (e.g., "PCM_16", "FLAC")
  builtin_interfaces/Time duration # Duration of audio
}

# Speech recognition result
message SpeechRecognitionResult {
  std_msgs/Header header
  string text                # Recognized text
  float64 confidence        # Recognition confidence
  float64 processing_time   # Time taken for recognition
  string language_code      # Detected language
  builtin_interfaces/Time timestamp # When recognition completed
}
```

## Capstone Project Data Models

### Autonomous Humanoid System

```
AutonomousHumanoidSystem {
  speech_recognition: SpeechRecognitionModule
  language_understanding: LanguageUnderstandingModule
  planning_system: PlanningModule
  execution_system: ExecutionModule
  monitoring_system: MonitoringModule
  safety_system: SafetyModule
  human_interface: HumanInterfaceModule
}

SpeechRecognitionModule {
  active_model: string                # Currently active ASR model
  confidence_threshold: float64       # Minimum confidence for acceptance
  wake_word_detection: WakeWordConfig # Configuration for wake word detection
  noise_suppression: NoiseSuppressionConfig # Noise suppression settings
}

LanguageUnderstandingModule {
  llm_model: string                   # Active LLM model
  intent_classifier: IntentClassifier # Intent classification model
  entity_extractor: EntityExtractor   # Entity extraction model
  context_manager: ContextManager     # Context management system
}

PlanningModule {
  task_planner: TaskPlanner           # High-level task planner
  motion_planner: MotionPlanner       # Low-level motion planner
  knowledge_base: KnowledgeBase       # World knowledge and constraints
  plan_validator: PlanValidator       # Plan validation system
}
```

## Configuration Data Models

### System Configuration

```yaml
vla_system:
  speech_recognition:
    model: "whisper-large"            # ASR model to use
    sample_rate: 16000               # Audio sample rate
    confidence_threshold: 0.7        # Minimum recognition confidence
    wake_word: "hey robot"           # Wake word for activation
    audio_device: "default"          # Audio input device

  language_model:
    model: "gpt-4"                   # LLM model to use
    temperature: 0.3                 # Sampling temperature
    max_tokens: 500                  # Maximum response tokens
    api_key: "YOUR_API_KEY"          # API key (or local model path)
    system_prompt: "You are a helpful robot assistant..."  # System prompt

  planning:
    max_plan_steps: 50               # Maximum steps in a plan
    timeout_per_action: 30.0         # Timeout for each action (seconds)
    retry_attempts: 3                # Number of retry attempts
    safety_constraints:              # Safety constraints
      max_velocity: 0.5
      max_acceleration: 1.0
      collision_threshold: 0.1

  execution:
    monitoring_frequency: 10.0       # Monitoring frequency (Hz)
    feedback_timeout: 5.0            # Feedback timeout (seconds)
    emergency_stop: true             # Enable emergency stop
    human_override: true             # Allow human override
```

These data models provide the foundation for consistent implementation of the Vision-Language-Action concepts covered in Module 4, ensuring proper integration between speech recognition, LLM processing, and robotic control systems.