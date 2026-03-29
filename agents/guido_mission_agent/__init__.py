try:
    from .agent import root_agent
except ModuleNotFoundError as exc:
    if exc.name and exc.name.startswith('google'):
        root_agent = None
    else:
        raise


__all__ = ['root_agent']
