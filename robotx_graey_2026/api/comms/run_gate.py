"""Competition-only verified RunStart admission; underwater loss does not revoke a run."""
import math


class RunGate:
    """match_run=False (the SE competition dive, 13 Nov 2026): the run ID is not compared.
    The first fresh, confirmed RUNNING admits; nothing afterwards revokes, because a
    different or ended run ID is no longer evidence about this mission. SA, pilot takeover,
    Cube failsafes and the mission's own limits still end it."""
    def __init__(self, competition=False, match_run=True):
        self.competition=competition
        self.match_run=match_run
        self.run_id=None
        self.sample={}
        self.received=-math.inf
        self.revoked=False

    def update(self,data,now):
        if not isinstance(data,dict):raise ValueError('communications status must be an object')
        stamp=data.get('published_monotonic',now)
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp>now:
            raise ValueError('invalid communications timestamp')
        self.sample,self.received=data,stamp
        run=data.get('remote_run_id',data.get('run_id'))
        if not self.match_run:return
        if self.run_id is not None and 0<=now-stamp<2 and ((data.get('remote_state') in ('ENDED','DECLARED') and run==self.run_id)
                or (data.get('run_confirmed') is True and run!=self.run_id)):
            self.revoked=True

    def admit(self,now,ready):
        if not self.competition:return True
        if self.revoked:return False
        if self.run_id is not None:return True
        s=self.sample;run=s.get('run_id')
        if (ready and 0<=now-self.received<2 and s.get('run_confirmed') is True
                and s.get('remote_state')=='RUNNING' and type(run) is int and 0<=run<=0xFFFFFFFF):
            self.run_id=run
            return True
        return False
