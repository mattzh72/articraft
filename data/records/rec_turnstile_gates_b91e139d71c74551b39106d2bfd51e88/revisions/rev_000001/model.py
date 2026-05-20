from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    frame_gray = model.material("frame_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    fixed_frame = model.part("fixed_frame")
    fixed_frame.visual(
        Box((1.28, 1.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=frame_gray,
        name="floor_plinth",
    )
    fixed_frame.visual(
        Box((0.22, 0.32, 1.34)),
        origin=Origin(xyz=(-0.56, 0.0, 0.67)),
        material=frame_gray,
        name="left_pylon",
    )
    fixed_frame.visual(
        Box((0.22, 0.32, 1.34)),
        origin=Origin(xyz=(0.56, 0.0, 0.67)),
        material=frame_gray,
        name="right_pylon",
    )
    fixed_frame.visual(
        Box((1.34, 0.32, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        material=frame_gray,
        name="header_beam",
    )
    fixed_frame.visual(
        Cylinder(radius=0.115, length=0.84),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=frame_gray,
        name="center_column",
    )
    fixed_frame.visual(
        Cylinder(radius=0.16, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=steel,
        name="bearing_housing",
    )
    fixed_frame.inertial = Inertial.from_geometry(
        Box((1.34, 1.04, 1.44)),
        mass=210.0,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.05, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=steel,
        name="rotor_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.11, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=steel,
        name="hub_drum",
    )

    arm_height = 0.08
    arm_length = 0.40
    arm_radius = 0.024
    grip_length = 0.12
    grip_radius = 0.029
    rotor.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(
            xyz=(0.5 * arm_length, 0.0, arm_height),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="arm_forward",
    )
    rotor.visual(
        Cylinder(radius=grip_radius, length=grip_length),
        origin=Origin(
            xyz=(arm_length - 0.06, 0.0, arm_height),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=rubber,
        name="grip_forward",
    )
    rotor.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(
            xyz=(0.5 * arm_length * cos(2.0 * pi / 3.0), 0.5 * arm_length * sin(2.0 * pi / 3.0), arm_height),
            rpy=(0.0, pi / 2.0, 2.0 * pi / 3.0),
        ),
        material=steel,
        name="arm_left_rear",
    )
    rotor.visual(
        Cylinder(radius=grip_radius, length=grip_length),
        origin=Origin(
            xyz=((arm_length - 0.06) * cos(2.0 * pi / 3.0), (arm_length - 0.06) * sin(2.0 * pi / 3.0), arm_height),
            rpy=(0.0, pi / 2.0, 2.0 * pi / 3.0),
        ),
        material=rubber,
        name="grip_left_rear",
    )
    rotor.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(
            xyz=(0.5 * arm_length * cos(4.0 * pi / 3.0), 0.5 * arm_length * sin(4.0 * pi / 3.0), arm_height),
            rpy=(0.0, pi / 2.0, 4.0 * pi / 3.0),
        ),
        material=steel,
        name="arm_right_rear",
    )
    rotor.visual(
        Cylinder(radius=grip_radius, length=grip_length),
        origin=Origin(
            xyz=((arm_length - 0.06) * cos(4.0 * pi / 3.0), (arm_length - 0.06) * sin(4.0 * pi / 3.0), arm_height),
            rpy=(0.0, pi / 2.0, 4.0 * pi / 3.0),
        ),
        material=rubber,
        name="grip_right_rear",
    )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.22),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=fixed_frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.01)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    fixed_frame = object_model.get_part("fixed_frame")
    rotor = object_model.get_part("rotor")
    hub_spin = object_model.get_articulation("hub_spin")

    ctx.expect_overlap(
        rotor,
        fixed_frame,
        axes="xy",
        elem_a="rotor_shaft",
        elem_b="bearing_housing",
        min_overlap=0.10,
        name="rotor shaft stays centered over the bearing housing",
    )
    ctx.expect_gap(
        rotor,
        fixed_frame,
        axis="z",
        positive_elem="rotor_shaft",
        negative_elem="bearing_housing",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotor shaft seats on top of the fixed bearing housing",
    )
    ctx.expect_gap(
        fixed_frame,
        rotor,
        axis="x",
        positive_elem="right_pylon",
        negative_elem="arm_forward",
        min_gap=0.02,
        name="right-facing arm clears the right pylon",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_arm_center = _center_from_aabb(ctx.part_element_world_aabb(rotor, elem="arm_forward"))
    with ctx.pose({hub_spin: pi / 3.0}):
        turned_arm_center = _center_from_aabb(ctx.part_element_world_aabb(rotor, elem="arm_forward"))
        ctx.expect_overlap(
            rotor,
            fixed_frame,
            axes="xy",
            elem_a="rotor_shaft",
            elem_b="bearing_housing",
            min_overlap=0.10,
            name="rotor shaft stays centered while the hub spins",
        )
    ctx.check(
        "arm rotates around the vertical center axis",
        rest_arm_center is not None
        and turned_arm_center is not None
        and turned_arm_center[1] > rest_arm_center[1] + 0.14
        and abs(turned_arm_center[2] - rest_arm_center[2]) < 0.01,
        details=f"rest={rest_arm_center}, turned={turned_arm_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
