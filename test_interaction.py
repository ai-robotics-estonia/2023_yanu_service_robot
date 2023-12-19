from hypothesis import given, strategies as st

# hello
from .sim_interaction import STATES


state_strategy = st.sampled_from(STATES)


@given(state_strategy)
async def test_callback_impl(states):
    sim_interaction = SimInteraction()
    # Add any necessary assertions here
    await sim_interaction.callback_impl(states)
