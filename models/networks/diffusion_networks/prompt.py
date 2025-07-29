# models/networks/diffusion_networks/prompt.py
import torch.nn as nn, torch

class SoftPrompt3D(nn.Module):
    def __init__(self, m_tokens=8, d_model=1280):
        super().__init__()
        self.bank = nn.Parameter(
            torch.randn(m_tokens, d_model) * 0.02  # small init
        )

    def forward(self, batch_size:int):
        return self.bank.unsqueeze(0).expand(batch_size, -1, -1)
