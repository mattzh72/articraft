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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_hand_wash_faucet")

    stainless = model.material("stainless", rgba=(0.80, 0.82, 0.84, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_grip = model.material("dark_grip", rgba=(0.19, 0.20, 0.22, 1.0))

    body = model.part("body")

    plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.180, 0.080, 0.012, corner_segments=8),
            0.006,
            cap=True,
            center=True,
        ),
        "mounting_plate",
    )
    body.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="mounting_plate",
    )

    body.visual(
        Cylinder(radius=0.0065, length=0.003),
        origin=Origin(xyz=(0.0, 0.0065, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="upper_screw_cap",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.003),
        origin=Origin(xyz=(0.0, 0.0065, -0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="lower_screw_cap",
    )

    body.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="escutcheon",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.036),
        origin=Origin(xyz=(0.0, 0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="valve_body",
    )
    body.visual(
        Cylinder(radius=0.0125, length=0.024),
        origin=Origin(xyz=(0.034, 0.024, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="handle_boss",
    )

    spout_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.024, 0.006),
                (0.0, 0.024, 0.115),
                (0.0, 0.026, 0.178),
                (0.0, 0.055, 0.185),
                (0.0, 0.086, 0.184),
                (0.0, 0.091, 0.148),
            ],
            radius=0.0115,
            samples_per_segment=16,
            radial_segments=20,
            cap_ends=False,
        ),
        "spout_pipe",
    )
    body.visual(
        spout_mesh,
        material=stainless,
        name="spout_pipe",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.180, 0.110, 0.205)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.040, 0.090)),
    )

    handle = model.part("handle")
    handle_paddle_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.016, 0.086, 0.006, corner_segments=8),
            0.010,
            cap=True,
            center=True,
        ),
        "handle_paddle",
    )
    handle.visual(
        Cylinder(radius=0.0135, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="handle_hub",
    )
    handle.visual(
        handle_paddle_mesh,
        origin=Origin(xyz=(0.020, 0.043, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="handle_paddle",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.020, 0.086, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grip,
        name="handle_tip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.032, 0.098, 0.030)),
        mass=0.16,
        origin=Origin(xyz=(0.016, 0.046, -0.002)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.046, 0.024, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.35,
            upper=0.90,
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
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    handle_joint = object_model.get_articulation("body_to_handle")

    ctx.expect_contact(
        handle,
        body,
        elem_a="handle_hub",
        elem_b="handle_boss",
        name="handle hub seats against valve boss",
    )
    ctx.expect_origin_gap(
        handle,
        body,
        axis="x",
        min_gap=0.040,
        max_gap=0.052,
        name="handle is mounted off the faucet right side",
    )

    valve_aabb = ctx.part_element_world_aabb(body, elem="valve_body")
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_pipe")
    ctx.check(
        "spout rises above valve body and reaches forward",
        valve_aabb is not None
        and spout_aabb is not None
        and spout_aabb[1][2] > valve_aabb[1][2] + 0.12
        and spout_aabb[1][1] > valve_aabb[1][1] + 0.04,
        details=f"valve_aabb={valve_aabb}, spout_aabb={spout_aabb}",
    )

    def _aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    rest_tip_aabb = ctx.part_element_world_aabb(handle, elem="handle_tip")
    with ctx.pose({handle_joint: 0.75}):
        ctx.expect_contact(
            handle,
            body,
            elem_a="handle_hub",
            elem_b="handle_boss",
            name="handle remains supported on valve boss when opened",
        )
        open_tip_aabb = ctx.part_element_world_aabb(handle, elem="handle_tip")

    rest_tip_center = _aabb_center(rest_tip_aabb) if rest_tip_aabb is not None else None
    open_tip_center = _aabb_center(open_tip_aabb) if open_tip_aabb is not None else None
    ctx.check(
        "handle lifts upward when opened",
        rest_tip_center is not None
        and open_tip_center is not None
        and open_tip_center[2] > rest_tip_center[2] + 0.03,
        details=f"rest_tip_center={rest_tip_center}, open_tip_center={open_tip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
