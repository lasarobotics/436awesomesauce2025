# 436 (Purplexity) Robot Code

created using finite state machines (courtesy of purplelib) and yagsl

### notes
- the fsm library is nice to use - there's just a lot of repetition. if it was cleaned up and provided separately from the rest of purplelib, it would be remarkably good (i.e. if StateMachine provided setState). it would also remove the need for certain things such as setting this.nextState to the same state given to the super() every time.