"""Distinguish overlapping bridge publishers from a sequential replacement."""


class BridgeIdentity:
    def __init__(self, freshness=.6):
        self.freshness = freshness
        self.seen = {}
        self.last = None

    def observe(self, instance, now):
        if not isinstance(instance, str) or not instance:
            return 'Bridge identity missing or invalid'
        self.seen = {key: stamp for key, stamp in self.seen.items()
                     if 0 <= now-stamp <= self.freshness}
        self.seen[instance] = now
        if len(self.seen) > 1:
            return 'Multiple bridge identities observed; duplicate or overlapping restart; revalidate external frame'
        changed = self.last is not None and instance != self.last
        self.last = instance
        return ('Navigation bridge restarted; external frame must be revalidated'
                if changed else '')
