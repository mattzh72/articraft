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
    model = ArticulatedObject(name="searchlight_tower")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.66, 1.0))
    mast_gray = model.material("mast_gray", rgba=(0.42, 0.45, 0.49, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.18, 0.20, 1.0))
    off_white = model.material("off_white", rgba=(0.86, 0.88, 0.83, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.82, 0.89, 0.96, 0.55))

    brace_geom = tube_from_spline_points(
        [
            (0.42, 0.0, 0.25),
            (0.34, 0.0, 0.82),
            (0.14, 0.0, 1.58),
        ],
        radius=0.03,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    brace_mesh = mesh_from_geometry(brace_geom, "tower_brace")

    handle_geom = tube_from_spline_points(
        [
            (-0.05, 0.0, 0.11),
            (-0.01, 0.0, 0.23),
            (0.18, 0.0, 0.23),
            (0.22, 0.0, 0.11),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    handle_mesh = mesh_from_geometry(handle_geom, "lamp_handle")

    tower_support = model.part("tower_support")
    tower_support.visual(
        Box((1.20, 1.20, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=concrete,
        name="foundation_block",
    )
    tower_support.visual(
        Box((0.32, 0.22, 0.46)),
        origin=Origin(xyz=(0.38, 0.0, 0.48)),
        material=mast_gray,
        name="service_cabinet",
    )
    tower_support.visual(
        Cylinder(radius=0.11, length=2.45),
        origin=Origin(xyz=(0.0, 0.0, 1.475)),
        material=mast_gray,
        name="mast_column",
    )
    tower_support.visual(
        Box((0.72, 0.72, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 2.725)),
        material=dark_metal,
        name="top_platform",
    )
    tower_support.visual(
        Cylinder(radius=0.18, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 2.825)),
        material=dark_metal,
        name="pan_bearing_collar",
    )
    for index in range(4):
        tower_support.visual(
            brace_mesh,
            origin=Origin(rpy=(0.0, 0.0, index * math.pi / 2.0)),
            material=mast_gray,
            name=f"brace_{index}",
        )
    tower_support.inertial = Inertial.from_geometry(
        Box((1.20, 1.20, 3.05)),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, 1.525)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.17, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_metal,
        name="turntable_drum",
    )
    pan_yoke.visual(
        Box((0.12, 0.44, 0.08)),
        origin=Origin(xyz=(-0.26, 0.0, 0.15)),
        material=mast_gray,
        name="yoke_base_block",
    )
    pan_yoke.visual(
        Box((0.10, 0.14, 0.10)),
        origin=Origin(xyz=(-0.18, 0.0, 0.15)),
        material=dark_metal,
        name="center_pedestal",
    )
    pan_yoke.visual(
        Box((0.28, 0.06, 0.52)),
        origin=Origin(xyz=(-0.14, 0.22, 0.405)),
        material=mast_gray,
        name="left_arm",
    )
    pan_yoke.visual(
        Box((0.28, 0.06, 0.52)),
        origin=Origin(xyz=(-0.14, -0.22, 0.405)),
        material=mast_gray,
        name="right_arm",
    )
    pan_yoke.visual(
        Cylinder(radius=0.03, length=0.44),
        origin=Origin(xyz=(-0.20, 0.0, 0.48), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_cross_tube",
    )
    pan_yoke.visual(
        Box((0.08, 0.44, 0.05)),
        origin=Origin(xyz=(-0.20, 0.0, 0.64)),
        material=dark_metal,
        name="top_bridge",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.44, 0.50, 0.66)),
        mass=48.0,
        origin=Origin(xyz=(-0.03, 0.0, 0.33)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.18, length=0.48),
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="main_body",
    )
    lamp_head.visual(
        Cylinder(radius=0.205, length=0.05),
        origin=Origin(xyz=(0.385, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.16, length=0.008),
        origin=Origin(xyz=(0.414, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(-0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_housing",
    )
    lamp_head.visual(
        Cylinder(radius=0.028, length=0.06),
        origin=Origin(xyz=(0.0, 0.16, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.028, length=0.06),
        origin=Origin(xyz=(0.0, -0.16, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    lamp_head.visual(
        handle_mesh,
        material=dark_metal,
        name="carry_handle",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.66),
        mass=22.0,
        origin=Origin(xyz=(0.09, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "support_to_pan",
        ArticulationType.REVOLUTE,
        parent=tower_support,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 2.925)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.9,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=1.0,
            lower=math.radians(-30.0),
            upper=math.radians(60.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_support = object_model.get_part("tower_support")
    pan_yoke = object_model.get_part("pan_yoke")
    lamp_head = object_model.get_part("lamp_head")
    pan_joint = object_model.get_articulation("support_to_pan")

    ctx.expect_origin_distance(
        tower_support,
        pan_yoke,
        axes="xy",
        max_dist=1e-6,
        name="pan stage stays centered on tower axis",
    )
    ctx.expect_origin_distance(
        pan_yoke,
        lamp_head,
        axes="xy",
        max_dist=1e-6,
        name="tilt axis stays centered in the yoke",
    )
    ctx.expect_contact(
        lamp_head,
        pan_yoke,
        elem_a="left_trunnion",
        elem_b="left_arm",
        name="left trunnion is carried by left yoke arm",
    )
    ctx.expect_contact(
        lamp_head,
        pan_yoke,
        elem_a="right_trunnion",
        elem_b="right_arm",
        name="right trunnion is carried by right yoke arm",
    )
    with ctx.pose({pan_joint: math.radians(75.0)}):
        ctx.expect_origin_distance(
            tower_support,
            pan_yoke,
            axes="xy",
            max_dist=1e-6,
            name="pan rotation remains coaxial with mast",
        )
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
