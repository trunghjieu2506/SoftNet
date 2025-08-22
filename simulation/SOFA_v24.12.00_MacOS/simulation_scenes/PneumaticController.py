# PneumaticController.py (fixed)
import Sofa.Core
from Sofa.constants import Key

class PneumaticController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Get root node
        self.node = kwargs.get("node", None)
        if self.node is None:
            # Fallback if not passed explicitly
            try:
                self.node = self.getContext()
            except Exception:
                pass
        if self.node is None:
            raise RuntimeError("[PneumaticController] Could not get root node (pass node=rootNode when adding the controller).")

        # Find the 'Finger' node
        self.finger = self.node.getChild('Finger')
        if self.finger is None:
            raise RuntimeError("[PneumaticController] Cannot find child node 'Finger'.")

        # Try the user’s naming first: Cavity1/spc
        self.spc = None
        cav1 = self.finger.getChild('Cavity1')
        if cav1 is not None:
            self.spc = cav1.getObject('spc')

        # If not found, try legacy names: Cavity/SurfacePressureConstraint
        if self.spc is None:
            cav = self.finger.getChild('Cavity')
            if cav is not None:
                self.spc = cav.getObject('SurfacePressureConstraint')

        # Last resort: search all children for a SurfacePressureConstraint object
        if self.spc is None:
            for child in self.finger.getChildren():
                try:
                    # getObject by name if single object is present
                    maybe = child.getObject('SurfacePressureConstraint')
                    if maybe is not None:
                        self.spc = maybe
                        break
                except Exception:
                    pass

        if self.spc is None:
            raise RuntimeError("[PneumaticController] SurfacePressureConstraint not found. "
                               "Expected 'Finger/Cavity1/spc' or 'Finger/Cavity/SurfacePressureConstraint'.")

        # Show initial value
        print(f"[PneumaticController] Connected to {self.spc.getName()} at path: "
              f"{self.spc.getPathName() if hasattr(self.spc, 'getPathName') else '(unknown path)'}")
        print(f"[PneumaticController] Initial pressure = {self._get_pressure():.5f}")

        # Limits
        self.min_pressure = 0.0
        self.max_pressure = 1.5
        self.step = 0.01

    def _get_pressure(self) -> float:
        """Return current pressure as float, whatever the internal Data layout."""
        try:
            v = self.spc.value.value  # often a list-like
            if isinstance(v, (list, tuple)):
                return float(v[0])
            return float(v)
        except Exception:
            try:
                # Another API variant
                return float(self.spc.findData('value').value[0])
            except Exception:
                # Last fallback
                return float(self.spc.value)

    def _set_pressure(self, p: float):
        """Set pressure, accepting scalar or list depending on the component."""
        p = float(p)
        try:
            self.spc.value = [p]
        except Exception:
            self.spc.value = p

    def onKeypressedEvent(self, ev):
        key = ev.get("key", None)
        if key is None:
            return

        p = self._get_pressure()

        if key == Key.plus or key == Key.equal:   # some keyboards send '=' without Shift
            p = min(self.max_pressure, p + self.step)
            self._set_pressure(p)
            print(f"[PneumaticController] Pressure increased to {p:.2f}")

        elif key == Key.minus or key == Key.underscore:
            p = max(self.min_pressure, p - self.step)
            self._set_pressure(p)
            print(f"[PneumaticController] Pressure decreased to {p:.2f}")
