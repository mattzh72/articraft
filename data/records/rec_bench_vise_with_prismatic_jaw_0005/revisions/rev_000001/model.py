from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry, filename: str):
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _prism_along_x(
    profile_yz: list[tuple[float, float]],
    *,
    length: float,
    center_x: float,
):
    geom = ExtrudeGeometry.from_z0(
        [(-z, y) for y, z in profile_yz],
        length,
        cap=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    geom.translate(center_x - (length * 0.5), 0.0, 0.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drill_press_vise", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.42, 0.45, 0.47, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.63, 0.65, 0.67, 1.0))
    jaw_steel = model.material("jaw_steel", rgba=(0.30, 0.32, 0.34, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    base = model.part("base")

    rail_length = 0.170
    rail_center_x = 0.035
    rail_y = 0.026
    rail_base_half = 0.011
    rail_top_half = 0.007
    rail_bottom_z = 0.014
    rail_top_z = 0.026

    def rail_profile(y_center: float) -> list[tuple[float, float]]:
        return [
            (y_center - rail_base_half, rail_bottom_z),
            (y_center + rail_base_half, rail_bottom_z),
            (y_center + rail_top_half, rail_top_z),
            (y_center - rail_top_half, rail_top_z),
        ]

    left_rail_mesh = _save_mesh(
        _prism_along_x(rail_profile(rail_y), length=rail_length, center_x=rail_center_x),
        "vise_left_dovetail_rail.obj",
    )
    right_rail_mesh = _save_mesh(
        _prism_along_x(rail_profile(-rail_y), length=rail_length, center_x=rail_center_x),
        "vise_right_dovetail_rail.obj",
    )

    base.visual(
        Box((0.280, 0.112, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        left_rail_mesh,
        material=machined_steel,
        name="left_dovetail_rail",
    )
    base.visual(
        right_rail_mesh,
        material=machined_steel,
        name="right_dovetail_rail",
    )
    base.visual(
        Box((0.036, 0.096, 0.028)),
        origin=Origin(xyz=(-0.104, 0.0, 0.028)),
        material=cast_iron,
        name="fixed_jaw_body",
    )
    base.visual(
        Box((0.010, 0.082, 0.038)),
        origin=Origin(xyz=(-0.081, 0.0, 0.033)),
        material=jaw_steel,
        name="fixed_jaw_face",
    )
    base.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(-0.066, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="fixed_screw_boss",
    )
    base.visual(
        Cylinder(radius=0.0045, length=0.172),
        origin=Origin(xyz=(0.030, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="leadscrew",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.117, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="front_collar",
    )
    base.visual(
        Cylinder(radius=0.0045, length=0.064),
        origin=Origin(xyz=(0.124, 0.0, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_steel,
        name="handle_bar",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.012),
        origin=Origin(xyz=(0.124, -0.032, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_steel,
        name="left_handle_knob",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.012),
        origin=Origin(xyz=(0.124, 0.032, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_steel,
        name="right_handle_knob",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.280, 0.112, 0.070)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.044, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.045, 0.002)),
        material=cast_iron,
        name="left_skirt",
    )
    moving_jaw.visual(
        Box((0.044, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, -0.045, 0.002)),
        material=cast_iron,
        name="right_skirt",
    )
    moving_jaw.visual(
        Box((0.044, 0.104, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=cast_iron,
        name="bridge",
    )
    moving_jaw.visual(
        Box((0.016, 0.018, 0.015)),
        origin=Origin(xyz=(0.010, 0.0, 0.0025)),
        material=machined_steel,
        name="nut_block",
    )
    moving_jaw.visual(
        Box((0.034, 0.094, 0.022)),
        origin=Origin(xyz=(0.008, 0.0, 0.033)),
        material=cast_iron,
        name="jaw_body",
    )
    moving_jaw.visual(
        Box((0.008, 0.082, 0.028)),
        origin=Origin(xyz=(-0.020, 0.0, 0.024)),
        material=jaw_steel,
        name="moving_jaw_face",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.052, 0.104, 0.058)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=moving_jaw,
        origin=Origin(xyz=(-0.020, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.06,
            lower=0.0,
            upper=0.084,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    moving_jaw = object_model.get_part("moving_jaw")
    jaw_slide = object_model.get_articulation("jaw_slide")

    base_plate = base.get_visual("base_plate")
    left_rail = base.get_visual("left_dovetail_rail")
    right_rail = base.get_visual("right_dovetail_rail")
    fixed_face = base.get_visual("fixed_jaw_face")
    leadscrew = base.get_visual("leadscrew")

    left_skirt = moving_jaw.get_visual("left_skirt")
    right_skirt = moving_jaw.get_visual("right_skirt")
    bridge = moving_jaw.get_visual("bridge")
    nut_block = moving_jaw.get_visual("nut_block")
    moving_face = moving_jaw.get_visual("moving_jaw_face")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        moving_jaw,
        base,
        reason="central leadscrew intentionally nests through the moving jaw nut housing",
    )

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        moving_jaw,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=left_skirt,
        negative_elem=base_plate,
        name="left_skirt_seated_on_base",
    )
    ctx.expect_gap(
        moving_jaw,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=right_skirt,
        negative_elem=base_plate,
        name="right_skirt_seated_on_base",
    )
    ctx.expect_within(
        base,
        moving_jaw,
        axes="y",
        inner_elem=left_rail,
        outer_elem=bridge,
        name="left_dovetail_rail_under_bridge",
    )
    ctx.expect_within(
        base,
        moving_jaw,
        axes="y",
        inner_elem=right_rail,
        outer_elem=bridge,
        name="right_dovetail_rail_under_bridge",
    )
    ctx.expect_within(
        base,
        moving_jaw,
        axes="yz",
        inner_elem=leadscrew,
        outer_elem=nut_block,
        name="leadscrew_centered_in_nut_block",
    )
    ctx.expect_gap(
        moving_jaw,
        base,
        axis="x",
        min_gap=0.030,
        max_gap=0.034,
        positive_elem=moving_face,
        negative_elem=fixed_face,
        name="rest_jaw_opening",
    )

    with ctx.pose({jaw_slide: 0.080}):
        ctx.expect_gap(
            moving_jaw,
            base,
            axis="x",
            min_gap=0.110,
            max_gap=0.114,
            positive_elem=moving_face,
            negative_elem=fixed_face,
            name="open_jaw_opening",
        )
        ctx.expect_gap(
            moving_jaw,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=1e-6,
            positive_elem=left_skirt,
            negative_elem=base_plate,
            name="left_skirt_stays_seated_when_open",
        )
        ctx.expect_gap(
            moving_jaw,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=1e-6,
            positive_elem=right_skirt,
            negative_elem=base_plate,
            name="right_skirt_stays_seated_when_open",
        )
        ctx.expect_within(
            base,
            moving_jaw,
            axes="y",
            inner_elem=left_rail,
            outer_elem=bridge,
            name="left_rail_stays_under_bridge_when_open",
        )
        ctx.expect_within(
            base,
            moving_jaw,
            axes="y",
            inner_elem=right_rail,
            outer_elem=bridge,
            name="right_rail_stays_under_bridge_when_open",
        )
        ctx.expect_within(
            base,
            moving_jaw,
            axes="yz",
            inner_elem=leadscrew,
            outer_elem=nut_block,
            name="leadscrew_stays_centered_when_open",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
