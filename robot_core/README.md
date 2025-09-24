# Robot Core

- [Robot Core](#robot-core)
  - [How to use](#how-to-use)
  - [Packages](#packages)
    - [robot\_launch](#robot_launch)


## How to use

1. Build the container: `docker-compose build robot_core`
2. Start the container: `docker-compose up robot_core`
3. Interactive mode: `docker-compose run --rm robot_core bash`


## Packages

### robot_launch
The entrypoint package that launches the application

```sh
ros2 launch robot_launch teleop.launch.py
```
