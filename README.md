# iris-controls-v2

Lightweight greenhouse control firmware skeleton focused on:

- Modbus weather + CWT reads
- Minimal climate planning (HOLD/VENT + safety latches)
- Curtain control
- MQTT telemetry

## Design Rules

- Keep modules small and readable.
- Keep lock scope short (`LockGuard<Mutex>` around data copy only).
- Keep command intent in lifecycle; curtain motion executes in its own service thread.
- Keep transport loops in dedicated service threads.

## Threads

- `ModbusService`: polls weather and CWT sensors, writes snapshots into `SharedState`.
- `MqttService`: reconnects/publishes telemetry snapshots from `SharedState`.
- `CurtainService`: owns per-curtain motion state and reconciles desired targets.
- Main thread (`AppLifecycle`): supervisor progression, climate planning, target publication, telemetry trigger.

## Naming Convention

- Class/type names: `PascalCase`
- Methods and variables: `camelCase`
- Constants: `kPascalCase`
- Telemetry JSON keys: `snake_case`

## Current Scope

- Uses lightweight transport interfaces for Modbus and MQTT.
- Includes synthetic sensor fallback if Modbus transport is not attached.
- Keeps extension points for future write support in both services.
