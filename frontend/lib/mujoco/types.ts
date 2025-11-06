/**
 * MuJoCo-related types for state replay
 */

export interface MujocoState {
  qpos: number[];
  qvel: number[];
  time: number;
}
