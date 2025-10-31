// MuJoCo WASM Loader Wrapper
// This script loads the MuJoCo WASM module and exposes it to the global scope

(async function() {
  try {
    // Import the MuJoCo WASM module
    // eslint-disable-next-line @next/next/no-assign-module-variable
    const module = await import('/mujoco/dist/mujoco_wasm.js');

    // Expose loadMujoco to the global scope
    window.loadMujoco = module.default;

    console.log('✅ MuJoCo loader initialized');

    // Dispatch a custom event to notify that MuJoCo is ready
    window.dispatchEvent(new Event('mujoco-ready'));
  } catch (error) {
    console.error('❌ Failed to load MuJoCo:', error);
    window.dispatchEvent(new CustomEvent('mujoco-error', { detail: error }));
  }
})();
