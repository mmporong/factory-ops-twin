"""Factory operator extension and its native inspection surface."""
_instance = None


def get_instance():
    return _instance


def _set_instance(instance):
    global _instance
    _instance = instance


from .extension import Extension
