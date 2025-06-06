tsParticles.load("particles-js", {
  fullScreen: { enable: false },
  particles: {
    number: { value: 120 },
    size: { value: 1.8 },
    color: { value: "#bb86fc" },
    opacity: { value: 0.5 },
    move: { enable: true, speed: 0.3 },
    links: {
      enable: true,
      color: "#bb86fc",
      distance: 100,
      opacity: 0.3
    }
  },
  background: {
    color: "#000000",
    opacity: 0
  }
});