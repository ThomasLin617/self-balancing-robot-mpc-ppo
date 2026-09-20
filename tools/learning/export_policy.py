import argparse
import json
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

from .state import EstimatedState, load_config, observation


def tensor_array(tensor):
    return tensor.detach().cpu().numpy().astype(float).tolist()


def export(checkpoint, output, config_path=None):
    config = load_config(config_path or Path(checkpoint).parent / "config.json")
    model = PPO.load(checkpoint, device="cpu")
    layers = [layer for layer in model.policy.mlp_extractor.policy_net if hasattr(layer, "weight")]
    payload = {
        "format": "d15_mlp_v1",
        "observation_scales": config["observation_scales"],
        "observation_clip": config["observation_clip"],
        "torque_limit": config["torque_limit"],
        "hidden": [
            {"weights": tensor_array(layer.weight), "bias": tensor_array(layer.bias)}
            for layer in layers
        ],
        "action": {
            "weights": tensor_array(model.policy.action_net.weight),
            "bias": tensor_array(model.policy.action_net.bias),
        },
    }
    with open(output, "w") as stream:
        json.dump(payload, stream, separators=(",", ":"))

    parity = []
    for raw in ([0, 0, 0, 0, 0, 0], [0.4, -0.2, 0.1, 0.03, -0.2, 0.4], [-5, 5, -1, 1, 2, -2]):
        policy_input = np.asarray(raw, dtype=np.float32)
        action, _ = model.predict(policy_input, deterministic=True)
        parity.append({"observation": raw, "action": np.asarray(action, dtype=float).tolist()})
    with open(str(output) + ".parity.json", "w") as stream:
        json.dump(parity, stream, indent=2)
    with open(str(output) + ".parity.txt", "w") as stream:
        for item in parity:
            stream.write("observation " + " ".join(map(str, item["observation"])) + " action "
                         + " ".join(map(str, item["action"])) + "\n")
    with open(str(output) + ".observations.txt", "w") as stream:
        for yaw in [0.0, np.pi/2, np.pi-1e-8, -np.pi+1e-8]:
            for goal in [[2.4, -1.2], [-100, 100]]:
                state = EstimatedState(config)
                state.x, state.y, state.yaw = 2.0, -1.0, yaw
                state.velocity, state.pitch = .1, .03
                state.pitch_rate, state.yaw_rate = -.2, .4
                raw = [state.x, state.y, state.yaw, *goal, state.velocity,
                       state.pitch, state.pitch_rate, state.yaw_rate]
                expected = observation(state, goal, config)
                stream.write(' '.join(map(str, [*raw, *expected.tolist()])) + '\n')
    with open(str(output) + ".txt", "w") as stream:
        stream.write("d15_mlp_v1\n")
        stream.write("input 6\n")
        stream.write("hidden " + " ".join(str(layer.weight.shape[0]) for layer in layers) + "\n")
        stream.write("scales " + " ".join(map(str, config["observation_scales"])) + "\n")
        stream.write("clip " + str(config["observation_clip"]) + "\n")
        stream.write("torque_limit " + str(config["torque_limit"]) + "\n")
        for index, layer in enumerate(layers):
            stream.write("layer " + str(index) + " " + str(layer.weight.shape[0]) + " " + str(layer.weight.shape[1]) + "\n")
            for row in tensor_array(layer.weight):
                stream.write("row " + " ".join(map(str, row)) + "\n")
            stream.write("bias " + str(index) + " " + " ".join(map(str, tensor_array(layer.bias))) + "\n")
        weights = tensor_array(model.policy.action_net.weight)
        stream.write("action_weights\n")
        for row in weights:
            stream.write("row " + " ".join(map(str, row)) + "\n")
        stream.write("action_bias " + " ".join(map(str, tensor_array(model.policy.action_net.bias))) + "\n")
    with open(str(output) + '.scenario.txt', 'w') as stream:
        limits = config['success']
        stream.write('d15_goal_sequence_v1\n')
        stream.write(' '.join(str(limits[key]) for key in
                             ['radius', 'velocity', 'yaw_rate', 'pitch', 'hold_seconds']) + '\n')
        stream.write(f"{config['stages']['D']['episode_seconds']} {config['fall_angle']}\n")
        goals = config['evaluation']['presentation_goals']
        stream.write(str(len(goals)) + '\n')
        for goal in goals:
            stream.write(' '.join(map(str, goal)) + '\n')
    print("Exported", output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint")
    parser.add_argument("output")
    parser.add_argument("--config")
    args = parser.parse_args()
    export(args.checkpoint, args.output, args.config)


if __name__ == "__main__":
    main()
