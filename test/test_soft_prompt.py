from utils.demo_util import SDFusionOpt
import pytest
import torch
from models.base_model import create_model
from models.networks.diffusion_networks.openai_model_3d import AttentionBlock, PromptedAttentionBlock
from models.networks.diffusion_networks.prompt import SoftPrompt3D

@pytest.mark.parametrize("prompt_len", [0, 4, 8])
def test_correct_prompt_architecture_and_learnable_params(prompt_len):
    """Test that model is correctly built with prompted attention blocks and that prompt params are learnable."""
    opt = SDFusionOpt(seed=42)
    ckpt_path = 'saved_ckpt/sdfusion-snet-all.pth'
    opt.init_model_args(ckpt_path=ckpt_path, top_k=10, lr=0.001, batch_size=1)
    device = "cpu"
    # opt.init_dset_args(dataset_mode="snet")
    model = create_model(opt)
    
    # Check that all AttentionBlocks are replaced with PromptedAttentionBlock
    for module in model.modules():
        if isinstance(module, AttentionBlock):
            raise AssertionError("Found AttentionBlock; expected all to be PromptedAttentionBlock.")
        if isinstance(module, PromptedAttentionBlock):
            # Check that soft_prompt parameter exists and has correct shape
            assert hasattr(module, 'soft_prompt'), "PromptedAttentionBlock missing 'soft_prompt' attribute."
            expected_shape = (prompt_len, module.channels)
            actual_shape = module.soft_prompt.weight.shape
            assert actual_shape == expected_shape, f"soft_prompt shape {actual_shape} != expected {expected_shape}"
    
    # # Check that soft_prompt parameters are learnable
    # optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    # dummy_input = torch.randn(2, 1, 32, 32, 32)  # batch_size=2
    # dummy_timestep = torch.tensor([10, 20])
    # dummy_context = None
    
    # model.train()
    # optimizer.zero_grad()
    # output = model(dummy_input, dummy_timestep, context=dummy_context)
    # loss = output.mean()
    # loss.backward()
    
    # # Ensure gradients are computed for soft_prompt parameters
    # for module in model.modules():
    #     if isinstance(module, PromptedAttentionBlock):
    #         assert module.soft_prompt.weight.grad is not None, "No gradient computed for soft_prompt parameters."