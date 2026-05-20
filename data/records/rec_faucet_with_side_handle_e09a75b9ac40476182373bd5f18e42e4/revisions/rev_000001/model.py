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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_basin_faucet")

    chrome = model.material("chrome", rgba=(0.84, 0.86, 0.89, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.10, 0.10, 0.11, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.12, 0.12, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.033, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=gasket_black,
        name="deck_gasket",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=chrome,
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=chrome,
        name="lower_collar",
    )
    body.visual(
        Cylinder(radius=0.0215, length=0.274),
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
        material=chrome,
        name="main_column",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=chrome,
        name="valve_shell",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, -0.030, 0.275), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_mount_pad",
    )

    spout_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.272),
            (0.018, 0.0, 0.284),
            (0.060, 0.0, 0.286),
            (0.100, 0.0, 0.270),
            (0.114, 0.0, 0.250),
        ],
        radius=0.0125,
        samples_per_segment=16,
        radial_segments=22,
        cap_ends=True,
    )
    body.visual(
        mesh_from_geometry(spout_geom, "faucet_spout"),
        material=chrome,
        name="spout_neck",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.018),
        origin=Origin(xyz=(0.114, 0.0, 0.239)),
        material=chrome,
        name="aerator_shell",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.114, 0.0, 0.233)),
        material=shadow_black,
        name="aerator_outlet",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.150, 0.070, 0.315)),
        mass=2.3,
        origin=Origin(xyz=(0.050, 0.0, 0.158)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.0045, length=0.058),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="handle_stem",
    )
    handle.visual(
        Cylinder(radius=0.0075, length=0.022),
        origin=Origin(xyz=(0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="handle_paddle",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.095, 0.020, 0.040)),
        mass=0.18,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, -0.044, 0.275)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(42.0),
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
    lever_joint = object_model.get_articulation("body_to_handle")

    axis = lever_joint.axis
    ctx.check(
        "lever joint axis is horizontal",
        axis is not None and abs(axis[0]) < 1e-6 and abs(abs(axis[1]) - 1.0) < 1e-6 and abs(axis[2]) < 1e-6,
        details=f"axis={axis}",
    )

    body_pos = ctx.part_world_position(body)
    handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "handle is mounted on the faucet side",
        body_pos is not None and handle_pos is not None and handle_pos[1] < body_pos[1] - 0.02,
        details=f"body_pos={body_pos}, handle_pos={handle_pos}",
    )

    with ctx.pose({lever_joint: 0.0}):
        ctx.expect_contact(
            handle,
            body,
            elem_a="handle_hub",
            elem_b="handle_mount_pad",
            contact_tol=0.0015,
            name="handle hub seats against handle mount",
        )
        rest_tip_aabb = ctx.part_element_world_aabb(handle, elem="handle_paddle")

    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        ctx.expect_contact(
            handle,
            body,
            elem_a="handle_hub",
            elem_b="handle_mount_pad",
            contact_tol=0.0015,
            name="handle hub remains seated when opened",
        )
        open_tip_aabb = ctx.part_element_world_aabb(handle, elem="handle_paddle")

    ctx.check(
        "positive handle rotation lifts the lever tip",
        rest_tip_aabb is not None
        and open_tip_aabb is not None
        and open_tip_aabb[1][2] > rest_tip_aabb[1][2] + 0.02,
        details=f"rest_tip_aabb={rest_tip_aabb}, open_tip_aabb={open_tip_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
