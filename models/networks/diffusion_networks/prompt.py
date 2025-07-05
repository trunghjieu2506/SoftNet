# models/networks/diffusion_networks/prompt.py
import torch.nn as nn, torch

class SoftPrompt3D(nn.Module):
    def __init__(self, m_tokens=8, d_model=1280):
        super().__init__()
        self.bank = nn.Parameter(
            torch.randn(m_tokens, d_model) * 0.02  # small init
        )

    def forward(self, k, v, B):
        # repeat prompt across batch and concat to K,V
        p = self.bank.unsqueeze(0).expand(B, -1, -1)   # (B, m, d)
        k = torch.cat([k, p], dim=1)
        v = torch.cat([v, p], dim=1)
        return k, v
