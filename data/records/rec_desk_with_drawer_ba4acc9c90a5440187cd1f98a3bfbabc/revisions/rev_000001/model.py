from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="reception_front_desk")

    painted_panel = model.material("painted_panel", rgba=(0.86, 0.84, 0.80, 1.0))
    oak_top = model.material("oak_top", rgba=(0.55, 0.40, 0.27, 1.0))
    laminate_inner = model.material("laminate_inner", rgba=(0.72, 0.72, 0.70, 1.0))
    drawer_face = model.material("drawer_face", rgba=(0.58, 0.43, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.61, 0.64, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.16, 0.17, 0.18, 1.0))

    desk_body = model.part("desk_body")
    desk_body.visual(
        Box((2.10, 0.06, 1.01)),
        origin=Origin(xyz=(0.0, -0.41, 0.505)),
        material=painted_panel,
        name="front_facade",
    )
    desk_body.visual(
        Box((2.10, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, -0.29, 1.03)),
        material=oak_top,
        name="counter_top",
    )
    desk_body.visual(
        Box((0.04, 0.74, 1.01)),
        origin=Origin(xyz=(-1.03, -0.08, 0.505)),
        material=painted_panel,
        name="left_end_panel",
    )
    desk_body.visual(
        Box((0.04, 0.88, 1.01)),
        origin=Origin(xyz=(1.03, 0.0, 0.505)),
        material=painted_panel,
        name="right_outer_panel",
    )
    desk_body.visual(
        Box((0.28, 0.58, 0.04)),
        origin=Origin(xyz=(0.89, 0.13, 1.03)),
        material=oak_top,
        name="return_top",
    )
    desk_body.visual(
        Box((0.28, 0.06, 1.01)),
        origin=Origin(xyz=(0.89, 0.39, 0.505)),
        material=painted_panel,
        name="return_back_panel",
    )
    desk_body.visual(
        Box((0.04, 0.18, 1.01)),
        origin=Origin(xyz=(0.75, 0.01, 0.505)),
        material=painted_panel,
        name="hinge_post",
    )
    desk_body.visual(
        Box((0.03, 0.24, 1.01)),
        origin=Origin(xyz=(0.135, -0.02, 0.505)),
        material=painted_panel,
        name="flap_latch_post",
    )
    desk_body.visual(
        Box((0.67, 0.56, 0.03)),
        origin=Origin(xyz=(-0.20, 0.14, 0.765)),
        material=laminate_inner,
        name="staff_worktop",
    )
    desk_body.visual(
        Box((0.03, 0.80, 0.76)),
        origin=Origin(xyz=(-0.53, 0.02, 0.38)),
        material=painted_panel,
        name="left_drawer_pedestal",
    )
    desk_body.visual(
        Box((0.03, 0.80, 0.76)),
        origin=Origin(xyz=(0.13, 0.02, 0.38)),
        material=painted_panel,
        name="right_drawer_pedestal",
    )
    desk_body.visual(
        Box((0.69, 0.02, 0.76)),
        origin=Origin(xyz=(-0.20, 0.43, 0.38)),
        material=painted_panel,
        name="rear_support_panel",
    )
    desk_body.visual(
        Box((0.03, 0.40, 0.05)),
        origin=Origin(xyz=(-0.47, 0.15, 0.725)),
        material=steel,
        name="left_rail",
    )
    desk_body.visual(
        Box((0.03, 0.40, 0.05)),
        origin=Origin(xyz=(0.07, 0.15, 0.725)),
        material=steel,
        name="right_rail",
    )
    desk_body.visual(
        Cylinder(radius=0.008, length=0.36),
        origin=Origin(xyz=(0.75, -0.098, 0.51)),
        material=dark_hardware,
        name="body_hinge_barrel",
    )
    desk_body.visual(
        Box((0.016, 0.012, 0.36)),
        origin=Origin(xyz=(0.754, -0.084, 0.51)),
        material=dark_hardware,
        name="body_hinge_leaf",
    )
    desk_body.inertial = Inertial.from_geometry(
        Box((2.10, 0.88, 1.05)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
    )

    entry_flap = model.part("entry_flap")
    entry_flap.visual(
        Box((0.592, 0.24, 0.04)),
        origin=Origin(xyz=(-0.304, 0.078, 0.52)),
        material=oak_top,
        name="flap_top",
    )
    entry_flap.visual(
        Box((0.592, 0.06, 0.91)),
        origin=Origin(xyz=(-0.304, -0.01, 0.045)),
        material=painted_panel,
        name="flap_apron",
    )
    entry_flap.visual(
        Cylinder(radius=0.008, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.27)),
        material=dark_hardware,
        name="flap_hinge_lower",
    )
    entry_flap.visual(
        Cylinder(radius=0.008, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=dark_hardware,
        name="flap_hinge_upper",
    )
    entry_flap.inertial = Inertial.from_geometry(
        Box((0.60, 0.24, 1.02)),
        mass=9.0,
        origin=Origin(xyz=(-0.30, 0.0, 0.0)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.62, 0.018, 0.19)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=drawer_face,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.012, 0.38, 0.15)),
        origin=Origin(xyz=(-0.274, -0.208, -0.02)),
        material=laminate_inner,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((0.012, 0.38, 0.15)),
        origin=Origin(xyz=(0.274, -0.208, -0.02)),
        material=laminate_inner,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((0.016, 0.40, 0.03)),
        origin=Origin(xyz=(-0.274, -0.208, 0.055)),
        material=steel,
        name="drawer_left_runner",
    )
    drawer.visual(
        Box((0.016, 0.40, 0.03)),
        origin=Origin(xyz=(0.274, -0.208, 0.055)),
        material=steel,
        name="drawer_right_runner",
    )
    drawer.visual(
        Box((0.548, 0.012, 0.15)),
        origin=Origin(xyz=(0.0, -0.392, -0.02)),
        material=laminate_inner,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.548, 0.38, 0.012)),
        origin=Origin(xyz=(0.0, -0.208, -0.089)),
        material=laminate_inner,
        name="drawer_bottom",
    )
    drawer.visual(
        Cylinder(radius=0.009, length=0.24),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.62, 0.42, 0.19)),
        mass=6.5,
        origin=Origin(xyz=(0.0, -0.20, -0.005)),
    )

    model.articulation(
        "desk_to_entry_flap",
        ArticulationType.REVOLUTE,
        parent=desk_body,
        child=entry_flap,
        origin=Origin(xyz=(0.75, -0.098, 0.51)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "desk_to_drawer",
        ArticulationType.PRISMATIC,
        parent=desk_body,
        child=drawer,
        origin=Origin(xyz=(-0.20, 0.38, 0.63)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.35,
            lower=0.0,
            upper=0.24,
        ),
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

    desk_body = object_model.get_part("desk_body")
    entry_flap = object_model.get_part("entry_flap")
    drawer = object_model.get_part("drawer")
    flap_joint = object_model.get_articulation("desk_to_entry_flap")
    drawer_joint = object_model.get_articulation("desk_to_drawer")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    with ctx.pose({flap_joint: 0.0, drawer_joint: 0.0}):
        ctx.expect_contact(
            entry_flap,
            desk_body,
            elem_a="flap_hinge_lower",
            elem_b="body_hinge_barrel",
            name="lower flap hinge barrel is mounted to the desk hinge barrel",
        )
        ctx.expect_contact(
            entry_flap,
            desk_body,
            elem_a="flap_hinge_upper",
            elem_b="body_hinge_barrel",
            name="upper flap hinge barrel is mounted to the desk hinge barrel",
        )
        ctx.expect_contact(
            entry_flap,
            desk_body,
            elem_a="flap_apron",
            elem_b="flap_latch_post",
            name="closed flap meets the latch post",
        )
        ctx.expect_gap(
            drawer,
            desk_body,
            axis="y",
            positive_elem="drawer_front",
            negative_elem="front_facade",
            min_gap=0.70,
            name="drawer front sits behind the customer-facing counter facade",
        )
        ctx.expect_gap(
            desk_body,
            drawer,
            axis="z",
            positive_elem="staff_worktop",
            negative_elem="drawer_front",
            min_gap=0.02,
            name="drawer clears the underside of the staff worktop",
        )
        ctx.expect_overlap(
            drawer,
            desk_body,
            axes="xy",
            elem_a="drawer_left_runner",
            elem_b="left_rail",
            min_overlap=0.01,
            name="drawer left side stays aligned with the left guide rail at rest",
        )
        ctx.expect_overlap(
            drawer,
            desk_body,
            axes="xy",
            elem_a="drawer_right_runner",
            elem_b="right_rail",
            min_overlap=0.01,
            name="drawer right side stays aligned with the right guide rail at rest",
        )
        ctx.expect_contact(
            drawer,
            desk_body,
            elem_a="drawer_left_runner",
            elem_b="left_rail",
            name="left drawer runner is carried by the fixed left rail",
        )
        ctx.expect_contact(
            drawer,
            desk_body,
            elem_a="drawer_right_runner",
            elem_b="right_rail",
            name="right drawer runner is carried by the fixed right rail",
        )

    closed_flap_center = _aabb_center(ctx.part_element_world_aabb(entry_flap, elem="flap_top"))
    drawer_rest_pos = ctx.part_world_position(drawer)

    with ctx.pose({flap_joint: 1.10}):
        open_flap_center = _aabb_center(ctx.part_element_world_aabb(entry_flap, elem="flap_top"))

    ctx.check(
        "entry flap swings inward for staff entry",
        closed_flap_center is not None
        and open_flap_center is not None
        and open_flap_center[1] > closed_flap_center[1] + 0.18,
        details=f"closed_center={closed_flap_center}, open_center={open_flap_center}",
    )

    with ctx.pose({drawer_joint: 0.24}):
        drawer_open_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            desk_body,
            axes="xy",
            elem_a="drawer_left_runner",
            elem_b="left_rail",
            min_overlap=0.01,
            name="drawer keeps left-rail alignment when extended",
        )
        ctx.expect_overlap(
            drawer,
            desk_body,
            axes="xy",
            elem_a="drawer_right_runner",
            elem_b="right_rail",
            min_overlap=0.01,
            name="drawer keeps right-rail alignment when extended",
        )
        ctx.expect_overlap(
            drawer,
            desk_body,
            axes="y",
            elem_a="drawer_left_runner",
            elem_b="left_rail",
            min_overlap=0.12,
            name="drawer retains insertion on the left rail when extended",
        )

    ctx.check(
        "drawer opens toward the staff side",
        drawer_rest_pos is not None
        and drawer_open_pos is not None
        and drawer_open_pos[1] > drawer_rest_pos[1] + 0.20,
        details=f"rest={drawer_rest_pos}, open={drawer_open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
