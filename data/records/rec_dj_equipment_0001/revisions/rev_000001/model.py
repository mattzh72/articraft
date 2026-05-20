from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _add_static_rotary(
    part,
    x: float,
    y: float,
    materials: dict[str, Material],
    radius: float = 0.011,
) -> None:
    part.visual(
        Cylinder(radius=radius + 0.0022, length=0.0018),
        origin=Origin(xyz=(x, y, 0.0410)),
        material=materials["trim"],
    )
    part.visual(
        Cylinder(radius=radius, length=0.010),
        origin=Origin(xyz=(x, y, 0.0458)),
        material=materials["rubber"],
    )
    part.visual(
        Cylinder(radius=radius * 0.30, length=0.0024),
        origin=Origin(xyz=(x, y, 0.0512)),
        material=materials["aluminum"],
    )
    part.visual(
        Box((0.0026, radius * 1.15, 0.0015)),
        origin=Origin(xyz=(x + radius * 0.42, y, 0.0515)),
        material=materials["aluminum"],
    )


def _add_pad(
    part,
    x: float,
    y: float,
    material: Material,
    size: tuple[float, float, float] = (0.021, 0.021, 0.0042),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=(x, y, 0.0426)),
        material=material,
    )
    part.visual(
        Box((size[0] * 0.76, size[1] * 0.76, 0.0015)),
        origin=Origin(xyz=(x, y, 0.0452)),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_controller", assets=ASSETS)

    materials = {
        "shell": _material("shell_black", (0.10, 0.11, 0.12, 1.0)),
        "panel": _material("panel_charcoal", (0.17, 0.18, 0.20, 1.0)),
        "trim": _material("trim_graphite", (0.26, 0.27, 0.29, 1.0)),
        "aluminum": _material("brushed_aluminum", (0.69, 0.71, 0.74, 1.0)),
        "rubber": _material("rubber_black", (0.06, 0.06, 0.07, 1.0)),
        "glass": _material("screen_glass", (0.23, 0.44, 0.57, 0.62)),
        "amber": _material("pad_amber", (0.78, 0.45, 0.16, 1.0)),
        "cyan": _material("pad_cyan", (0.18, 0.62, 0.76, 1.0)),
        "green": _material("play_green", (0.23, 0.63, 0.28, 1.0)),
        "orange": _material("cue_orange", (0.84, 0.49, 0.18, 1.0)),
        "internal": _material("internal_support", (0.0, 0.0, 0.0, 0.0)),
    }
    model.materials.extend(materials.values())

    width = 0.34
    depth = 0.22
    body_height = 0.04

    body_geom = superellipse_side_loft(
        [
            (-depth / 2.0, 0.0, 0.013, width * 0.95),
            (-0.082, 0.0, 0.018, width * 0.97),
            (-0.050, 0.0, 0.026, width * 0.985),
            (-0.010, 0.0, 0.036, width),
            (0.055, 0.0, body_height, width),
            (depth / 2.0, 0.0, body_height, width * 0.96),
        ],
        exponents=3.8,
        segments=52,
        cap=True,
        closed=True,
    )
    body_mesh = mesh_from_geometry(body_geom, ASSETS.mesh_path("dj_controller_body.obj"))

    body = model.part("body")
    body.visual(body_mesh, material=materials["shell"])
    body.inertial = Inertial.from_geometry(
        Box((width, depth, body_height)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    top_panel = model.part("top_panel")
    top_panel.visual(
        Box((0.312, 0.206, 0.054)),
        origin=Origin(xyz=(0.0, 0.005, 0.027)),
        material=materials["internal"],
    )
    top_panel.visual(
        Box((0.322, 0.172, 0.006)),
        origin=Origin(xyz=(0.0, 0.015, 0.038)),
        material=materials["panel"],
    )
    top_panel.inertial = Inertial.from_geometry(
        Box((0.322, 0.172, 0.018)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.015, 0.046)),
    )

    top_panel.visual(
        Box((0.078, 0.166, 0.0016)),
        origin=Origin(xyz=(0.0, 0.015, 0.0413)),
        material=materials["trim"],
    )
    top_panel.visual(
        Box((0.088, 0.029, 0.008)),
        origin=Origin(xyz=(0.0, 0.094, 0.046)),
        material=materials["trim"],
    )
    top_panel.visual(
        Box((0.080, 0.023, 0.0015)),
        origin=Origin(xyz=(0.0, 0.094, 0.0508)),
        material=materials["glass"],
    )
    top_panel.visual(
        Box((0.118, 0.010, 0.0016)),
        origin=Origin(xyz=(0.0, -0.086, 0.0413)),
        material=materials["trim"],
    )
    top_panel.visual(
        Box((0.012, 0.094, 0.0016)),
        origin=Origin(xyz=(0.0, -0.015, 0.0413)),
        material=materials["trim"],
    )
    top_panel.visual(
        Cylinder(radius=0.020, length=0.0022),
        origin=Origin(xyz=(0.0, 0.058, 0.0412)),
        material=materials["aluminum"],
    )

    for x_pos in (-0.105, 0.105):
        top_panel.visual(
            Cylinder(radius=0.046, length=0.007),
            origin=Origin(xyz=(x_pos, 0.028, 0.0440)),
            material=materials["rubber"],
        )
        top_panel.visual(
            Cylinder(radius=0.039, length=0.005),
            origin=Origin(xyz=(x_pos, 0.028, 0.0455)),
            material=materials["aluminum"],
        )
        top_panel.visual(
            Cylinder(radius=0.016, length=0.0020),
            origin=Origin(xyz=(x_pos, 0.028, 0.0490)),
            material=materials["trim"],
        )
        top_panel.visual(
            Box((0.003, 0.016, 0.0015)),
            origin=Origin(xyz=(x_pos + 0.027, 0.028, 0.0488)),
            material=materials["orange"],
        )

    for x_pos in (-0.036, 0.036):
        for y_pos in (0.048, 0.016, -0.016):
            _add_static_rotary(top_panel, x_pos, y_pos, materials)

    for x_pos in (-0.048, -0.022, 0.022, 0.048):
        top_panel.visual(
            Box((0.016, 0.0085, 0.003)),
            origin=Origin(xyz=(x_pos, 0.068, 0.0420)),
            material=materials["trim"],
        )

    for x_pos, mat in ((-0.052, materials["orange"]), (0.052, materials["green"])):
        top_panel.visual(
            Box((0.018, 0.018, 0.0042)),
            origin=Origin(xyz=(x_pos, -0.054, 0.0426)),
            material=mat,
        )

    for x_pos in (-0.124, -0.095, 0.095, 0.124):
        for y_pos, mat in ((-0.056, materials["amber"]), (-0.032, materials["cyan"])):
            _add_pad(top_panel, x_pos, y_pos, mat)

    model.articulation(
        "body_to_top_panel",
        ArticulationType.FIXED,
        parent="body",
        child="top_panel",
        origin=Origin(),
    )

    for x_pos in (-0.135, 0.135):
        for y_pos in (-0.082, 0.082):
            body.visual(
                Box((0.028, 0.022, 0.008)),
                origin=Origin(xyz=(x_pos, y_pos, 0.002)),
                material=materials["rubber"],
            )

    channel_fader = model.part("channel_fader")
    channel_fader.visual(
        Box((0.005, 0.060, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=materials["trim"],
    )
    channel_fader.visual(
        Box((0.015, 0.024, 0.013)),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=materials["aluminum"],
    )
    channel_fader.visual(
        Box((0.010, 0.019, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0133)),
        material=materials["rubber"],
    )
    channel_fader.inertial = Inertial.from_geometry(
        Box((0.015, 0.060, 0.013)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
    )

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.110, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=materials["trim"],
    )
    crossfader.visual(
        Box((0.034, 0.017, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=materials["aluminum"],
    )
    crossfader.visual(
        Box((0.022, 0.010, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0122)),
        material=materials["rubber"],
    )
    crossfader.inertial = Inertial.from_geometry(
        Box((0.110, 0.017, 0.012)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    browse_knob = model.part("browse_knob")
    browse_knob.visual(
        Cylinder(radius=0.005, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=materials["aluminum"],
    )
    browse_knob.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=materials["rubber"],
    )
    browse_knob.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=materials["rubber"],
    )
    browse_knob.visual(
        Cylinder(radius=0.0105, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0195)),
        material=materials["aluminum"],
    )
    browse_knob.visual(
        Box((0.004, 0.018, 0.0018)),
        origin=Origin(xyz=(0.009, 0.0, 0.0200)),
        material=materials["aluminum"],
    )
    browse_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.017, length=0.020),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "channel_fader_slide",
        ArticulationType.PRISMATIC,
        parent="top_panel",
        child="channel_fader",
        origin=Origin(xyz=(0.0, -0.015, 0.041)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.25,
            lower=-0.032,
            upper=0.032,
        ),
    )
    model.articulation(
        "crossfader_slide",
        ArticulationType.PRISMATIC,
        parent="top_panel",
        child="crossfader",
        origin=Origin(xyz=(0.0, -0.086, 0.041)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.30,
            lower=-0.055,
            upper=0.055,
        ),
    )
    model.articulation(
        "browse_knob_turn",
        ArticulationType.REVOLUTE,
        parent="top_panel",
        child="browse_knob",
        origin=Origin(xyz=(0.0, 0.058, 0.041)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.5,
            lower=-2.4,
            upper=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("channel_fader", "body", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_gap("channel_fader", "body", axis="z", max_gap=0.002, max_penetration=0.006)
    ctx.expect_origin_distance("channel_fader", "body", axes="xy", max_dist=0.035)

    ctx.expect_aabb_overlap("crossfader", "body", axes="xy", min_overlap=0.014)
    ctx.expect_aabb_gap("crossfader", "body", axis="z", max_gap=0.002, max_penetration=0.006)
    ctx.expect_origin_distance("crossfader", "body", axes="xy", max_dist=0.095)

    ctx.expect_aabb_overlap("browse_knob", "body", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("browse_knob", "body", axis="z", max_gap=0.006, max_penetration=0.006)
    ctx.expect_origin_distance("browse_knob", "body", axes="xy", max_dist=0.065)

    ctx.expect_joint_motion_axis(
        "channel_fader_slide",
        "channel_fader",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "crossfader_slide",
        "crossfader",
        world_axis="x",
        direction="positive",
        min_delta=0.03,
    )

    with ctx.pose(channel_fader_slide=-0.032):
        ctx.expect_aabb_overlap("channel_fader", "body", axes="xy", min_overlap=0.010)
        ctx.expect_aabb_gap("channel_fader", "body", axis="z", max_gap=0.002, max_penetration=0.006)
        ctx.expect_origin_distance("channel_fader", "body", axes="xy", max_dist=0.055)

    with ctx.pose(channel_fader_slide=0.032):
        ctx.expect_aabb_overlap("channel_fader", "body", axes="xy", min_overlap=0.010)
        ctx.expect_aabb_gap("channel_fader", "body", axis="z", max_gap=0.002, max_penetration=0.006)
        ctx.expect_origin_distance("channel_fader", "body", axes="xy", max_dist=0.055)

    with ctx.pose(crossfader_slide=-0.055):
        ctx.expect_aabb_overlap("crossfader", "body", axes="xy", min_overlap=0.014)
        ctx.expect_aabb_gap("crossfader", "body", axis="z", max_gap=0.002, max_penetration=0.006)
        ctx.expect_origin_distance("crossfader", "body", axes="xy", max_dist=0.115)

    with ctx.pose(crossfader_slide=0.055):
        ctx.expect_aabb_overlap("crossfader", "body", axes="xy", min_overlap=0.014)
        ctx.expect_aabb_gap("crossfader", "body", axis="z", max_gap=0.002, max_penetration=0.006)
        ctx.expect_origin_distance("crossfader", "body", axes="xy", max_dist=0.115)

    with ctx.pose(browse_knob_turn=-2.4):
        ctx.expect_aabb_overlap("browse_knob", "body", axes="xy", min_overlap=0.018)
        ctx.expect_aabb_gap("browse_knob", "body", axis="z", max_gap=0.006, max_penetration=0.006)
        ctx.expect_origin_distance("browse_knob", "body", axes="xy", max_dist=0.070)

    with ctx.pose(browse_knob_turn=2.4):
        ctx.expect_aabb_overlap("browse_knob", "body", axes="xy", min_overlap=0.018)
        ctx.expect_aabb_gap("browse_knob", "body", axis="z", max_gap=0.006, max_penetration=0.006)
        ctx.expect_origin_distance("browse_knob", "body", axes="xy", max_dist=0.070)

    with ctx.pose(
        channel_fader_slide=0.032,
        crossfader_slide=-0.055,
        browse_knob_turn=2.4,
    ):
        ctx.expect_aabb_overlap("channel_fader", "body", axes="xy", min_overlap=0.010)
        ctx.expect_aabb_overlap("crossfader", "body", axes="xy", min_overlap=0.014)
        ctx.expect_aabb_overlap("browse_knob", "body", axes="xy", min_overlap=0.018)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
