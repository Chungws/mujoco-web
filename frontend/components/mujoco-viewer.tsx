'use client';

import { useEffect, useRef, useState } from 'react';
import Script from 'next/script';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

interface MujocoViewerProps {
  className?: string;
}

// Declare global loadMujoco function
declare global {
  interface Window {
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    loadMujoco: any;
  }
}

export function MujocoViewer({ className }: MujocoViewerProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const [status, setStatus] = useState<'loading' | 'ready' | 'error'>('loading');
  const [error, setError] = useState<string | null>(null);
  const [mujocoLoaded, setMujocoLoaded] = useState(false);

  useEffect(() => {
    if (!containerRef.current || !mujocoLoaded) return;

    const container = containerRef.current;
    let mounted = true;
    let animationId: number;
    let scene: THREE.Scene;
    let camera: THREE.PerspectiveCamera;
    let renderer: THREE.WebGLRenderer;
    let controls: OrbitControls;

    const init = async () => {
      try {
        // Check WebAssembly support
        if (typeof WebAssembly !== 'object') {
          throw new Error('WebAssembly is not supported in this browser');
        }

        // Check if loadMujoco is available
        if (typeof window.loadMujoco === 'undefined') {
          throw new Error('MuJoCo WASM module not loaded');
        }

        // Load MuJoCo WASM
        console.log('🔄 Loading MuJoCo WASM...');
        const mujoco = await window.loadMujoco();

        if (!mounted) return;

        // Setup Three.js scene
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x264653);
        scene.fog = new THREE.Fog(scene.background, 15, 25.5);

        // Setup camera
        camera = new THREE.PerspectiveCamera(
          45,
          container.clientWidth / container.clientHeight,
          0.001,
          1000
        );
        camera.position.set(2.0, 1.7, 1.7);
        scene.add(camera);

        // Setup lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.3);
        scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 10, 7.5);
        directionalLight.castShadow = true;
        scene.add(directionalLight);

        // Setup renderer
        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setPixelRatio(window.devicePixelRatio);
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        container.appendChild(renderer.domElement);

        // Setup controls
        controls = new OrbitControls(camera, renderer.domElement);
        controls.target.set(0, 0.7, 0);
        controls.panSpeed = 2;
        controls.zoomSpeed = 1;
        controls.enableDamping = true;
        controls.dampingFactor = 0.10;
        controls.screenSpacePanning = true;
        controls.update();

        // Add a test cube to verify Three.js is working
        const geometry = new THREE.BoxGeometry(0.5, 0.5, 0.5);
        const material = new THREE.MeshStandardMaterial({
          color: 0x2a9d8f,
          roughness: 0.5,
          metalness: 0.5
        });
        const cube = new THREE.Mesh(geometry, material);
        cube.position.set(0, 0.5, 0);
        cube.castShadow = true;
        cube.receiveShadow = true;
        scene.add(cube);

        // Add ground plane
        const groundGeometry = new THREE.PlaneGeometry(20, 20);
        const groundMaterial = new THREE.MeshStandardMaterial({
          color: 0xe9c46a,
          side: THREE.DoubleSide,
          roughness: 0.8
        });
        const ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        ground.receiveShadow = true;
        scene.add(ground);

        // Setup MuJoCo virtual filesystem
        const workingPath = '/working';
        try {
          mujoco.FS.mkdir(workingPath);
        } catch (e) {
          const error = e as { code?: string };
          if (error?.code !== 'EEXIST') {
            console.warn('Failed to create /working directory:', e);
          }
        }
        try {
          mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, workingPath);
        } catch (e) {
          const error = e as { code?: string };
          if (error?.code !== 'EEXIST' && error?.code !== 'EBUSY') {
            console.warn('Failed to mount MEMFS at /working:', e);
          }
        }

        console.log('✅ MuJoCo WASM loaded successfully!');
        console.log('📦 MuJoCo version:', mujoco.mj_version());

        if (!mounted) return;
        setStatus('ready');

        // Animation loop
        const animate = () => {
          if (!mounted) return;
          animationId = requestAnimationFrame(animate);

          // Rotate the cube for visual feedback
          cube.rotation.x += 0.01;
          cube.rotation.y += 0.01;

          controls.update();
          renderer.render(scene, camera);
        };
        animate();

        // Handle window resize
        const handleResize = () => {
          camera.aspect = container.clientWidth / container.clientHeight;
          camera.updateProjectionMatrix();
          renderer.setSize(container.clientWidth, container.clientHeight);
        };
        window.addEventListener('resize', handleResize);

        return () => {
          window.removeEventListener('resize', handleResize);
        };
      } catch (err) {
        console.error('❌ Failed to initialize MuJoCo viewer:', err);
        if (!mounted) return;
        setError(err instanceof Error ? err.message : 'Unknown error');
        setStatus('error');
      }
    };

    init();

    return () => {
      mounted = false;
      if (animationId) {
        cancelAnimationFrame(animationId);
      }
      if (renderer && container) {
        container.removeChild(renderer.domElement);
        renderer.dispose();
      }
      if (controls) {
        controls.dispose();
      }
    };
  }, [mujocoLoaded]);

  return (
    <>
      <Script
        src="/mujoco-loader.js"
        type="module"
        strategy="afterInteractive"
        onLoad={() => {
          // Wait for the mujoco-ready event
          const handleReady = () => {
            console.log('✅ MuJoCo ready');
            setMujocoLoaded(true);
            window.removeEventListener('mujoco-ready', handleReady);
          };

          const handleError = ((e: CustomEvent) => {
            console.error('❌ MuJoCo error:', e.detail);
            setError('Failed to initialize MuJoCo');
            setStatus('error');
            window.removeEventListener('mujoco-error', handleError as EventListener);
          }) as EventListener;

          window.addEventListener('mujoco-ready', handleReady);
          window.addEventListener('mujoco-error', handleError);
        }}
        onError={(e) => {
          console.error('❌ Failed to load MuJoCo loader script:', e);
          setError('Failed to load MuJoCo loader script');
          setStatus('error');
        }}
      />

      <div className={`relative w-full h-full ${className || ''}`}>
        <div ref={containerRef} className="w-full h-full" />

        {status === 'loading' && (
          <div className="absolute inset-0 flex items-center justify-center bg-gradient-to-br from-slate-900 to-slate-800">
            <div className="text-center text-white">
              <div className="animate-spin rounded-full h-16 w-16 border-b-2 border-white mx-auto mb-4"></div>
              <p className="text-lg font-semibold">Loading MuJoCo 3.3.7...</p>
              <p className="text-sm text-slate-300 mt-2">Initializing WebAssembly</p>
            </div>
          </div>
        )}

        {status === 'error' && (
          <div className="absolute inset-0 flex items-center justify-center bg-gradient-to-br from-red-900 to-red-800">
            <div className="text-center text-white p-8 max-w-md bg-black/20 rounded-lg backdrop-blur-sm">
              <p className="text-2xl font-bold mb-4">⚠️ Error</p>
              <p className="text-sm mb-4">{error}</p>
              <button
                onClick={() => window.location.reload()}
                className="px-4 py-2 bg-white text-red-900 rounded-md hover:bg-slate-100 transition-colors"
              >
                Reload Page
              </button>
            </div>
          </div>
        )}
      </div>
    </>
  );
}
