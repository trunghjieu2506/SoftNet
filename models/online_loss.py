import mcubes, tempfile, subprocess, torch
# from utils.sofa_wrapper import run_sofa_once   # ← you wrap your earlier SOFA scene here

def sdf_to_bend(sdf_vol, pressure_bar=0.08):
    verts, faces = mcubes.marching_cubes(sdf_vol, 0.0)
    tmp = tempfile.mkdtemp()
    stl = f"{tmp}/geom.stl"
    mcubes.export_mesh(verts, faces, stl)
    msh = f"{tmp}/geom.msh"
    subprocess.run(['gmsh', stl, '-3', '-format', 'msh2', '-o', msh,
                    '-algo', 'DelQuad'], check=True, stdout=subprocess.DEVNULL)
    return 2
    # return run_sofa_once(msh, pressure_bar)      # returns bending angle (rad)

class OnlineLoss(torch.nn.Module):
    def __init__(self, sampler, vqvae):
        super().__init__()
        self.sampler = sampler
        self.vqvae   = vqvae
        self.baseline = None                     # REINFORCE baseline

    def forward(self, z_shape, ddim_steps=50, eta=0.0):
        latent, _ = self.sampler.sample(S=ddim_steps,
                                        batch_size=1,
                                        shape=z_shape,
                                        conditioning=None,
                                        eta=eta)
        sdf = self.vqvae.decode_no_quant(latent).squeeze().cpu().numpy()
        angle = sdf_to_bend(sdf)                 # scalar
        loss  = torch.tensor(angle, device=latent.device, requires_grad=True)
        if self.baseline is None:
            self.baseline = angle
        adv = loss - self.baseline
        self.baseline = 0.9*self.baseline + 0.1*angle
        return adv
