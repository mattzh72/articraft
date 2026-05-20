from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
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
    rounded_rect_profile,
    section_loft,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


BODY_WIDTH = 0.445
BODY_DEPTH = 0.312
BODY_HEIGHT = 0.079

GLASS_CENTER = (-0.048, -0.010)
GLASS_SIZE = (0.296, 0.228)
LID_CENTER_X = GLASS_CENTER[0]
LID_SIZE = (0.344, 0.278)
CONTROL_PANEL_SIZE = (0.092, 0.068)
CONTROL_PANEL_CENTER = (0.172, 0.101)


def _rounded_section(
    width: float,
    depth: float,
    z: float,
    radius: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x + center_x, y + center_y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flatbed_scanner", assets=ASSETS)

    warm_white = model.material("warm_white", rgba=(0.91, 0.92, 0.90, 1.0))
    light_liner = model.material("light_liner", rgba=(0.95, 0.95, 0.93, 1.0))
    frame_grey = model.material("frame_grey", rgba=(0.72, 0.74, 0.75, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.22, 0.24, 0.26, 1.0))
    black_gloss = model.material("black_gloss", rgba=(0.08, 0.09, 0.10, 1.0))
    button_grey = model.material("button_grey", rgba=(0.63, 0.66, 0.69, 1.0))
    power_blue = model.material("power_blue", rgba=(0.35, 0.54, 0.86, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.69, 0.82, 0.90, 0.38))
    glass_backing = model.material("glass_backing", rgba=(0.17, 0.20, 0.23, 0.88))

    body_geom = section_loft(
        [
            _rounded_section(BODY_WIDTH, BODY_DEPTH, 0.0, 0.028),
            _rounded_section(BODY_WIDTH, BODY_DEPTH, 0.012, 0.028),
            _rounded_section(0.434, 0.298, 0.068, 0.025, center_y=-0.004),
            _rounded_section(0.426, 0.292, BODY_HEIGHT, 0.023, center_y=-0.006),
        ]
    )
    body_mesh = _mesh("scanner_body.obj", body_geom)

    base = model.part("base")
    base.visual(body_mesh, material=warm_white)

    base.visual(
        Box((0.336, 0.017, 0.006)),
        origin=Origin(xyz=(GLASS_CENTER[0], 0.1145, BODY_HEIGHT - 0.004)),
        material=frame_grey,
        name="front_bezel",
    )
    base.visual(
        Box((0.336, 0.017, 0.006)),
        origin=Origin(xyz=(GLASS_CENTER[0], -0.1345, BODY_HEIGHT - 0.004)),
        material=frame_grey,
        name="rear_bezel",
    )
    base.visual(
        Box((0.017, 0.228, 0.006)),
        origin=Origin(xyz=(-0.2085, GLASS_CENTER[1], BODY_HEIGHT - 0.004)),
        material=frame_grey,
        name="left_bezel",
    )
    base.visual(
        Box((0.017, 0.228, 0.006)),
        origin=Origin(xyz=(0.1125, GLASS_CENTER[1], BODY_HEIGHT - 0.004)),
        material=frame_grey,
        name="right_bezel",
    )
    base.visual(
        Box((0.264, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.145, BODY_HEIGHT - 0.006)),
        material=dark_trim,
        name="hinge_rail",
    )
    for side_x in (-0.117, 0.117):
        base.visual(
            Cylinder(radius=0.008, length=0.074),
            origin=Origin(
                xyz=(side_x, -0.147, BODY_HEIGHT - 0.001),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_trim,
            name=f"base_hinge_barrel_{'l' if side_x < 0.0 else 'r'}",
        )
    for side_x in (-0.170, 0.170):
        for side_y in (-0.116, 0.116):
            base.visual(
                Box((0.030, 0.024, 0.004)),
                origin=Origin(xyz=(side_x, side_y, 0.002)),
                material=rubber,
                name=f"foot_{'l' if side_x < 0.0 else 'r'}_{'rear' if side_y < 0.0 else 'front'}",
            )
    base.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=3.9,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    scan_glass = model.part("scan_glass")
    scan_glass.visual(
        Box((0.312, 0.244, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.0005)),
        material=glass_backing,
        name="platen_backing",
    )
    scan_glass.visual(
        Box((GLASS_SIZE[0], GLASS_SIZE[1], 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=glass,
        name="glass_plate",
    )
    scan_glass.inertial = Inertial.from_geometry(
        Box((0.312, 0.244, 0.007)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
    )

    panel_geom = section_loft(
        [
            _rounded_section(CONTROL_PANEL_SIZE[0], CONTROL_PANEL_SIZE[1], 0.0, 0.010),
            _rounded_section(CONTROL_PANEL_SIZE[0], CONTROL_PANEL_SIZE[1], 0.005, 0.010),
            _rounded_section(0.080, 0.052, 0.014, 0.008, center_y=-0.003),
        ]
    )
    panel_mesh = _mesh("scanner_control_panel.obj", panel_geom)

    control_panel = model.part("control_panel")
    control_panel.visual(panel_mesh, material=dark_trim)
    control_panel.visual(
        Box((0.036, 0.016, 0.003)),
        origin=Origin(xyz=(-0.020, 0.006, 0.013)),
        material=black_gloss,
        name="status_display",
    )
    control_panel.visual(
        Cylinder(radius=0.008, length=0.0035),
        origin=Origin(xyz=(0.025, -0.013, 0.013)),
        material=power_blue,
        name="power_button",
    )
    for idx, button_x in enumerate((0.002, 0.018, 0.034)):
        control_panel.visual(
            Cylinder(radius=0.0055, length=0.003),
            origin=Origin(xyz=(button_x, 0.009, 0.0125)),
            material=button_grey,
            name=f"function_button_{idx + 1}",
        )
    control_panel.visual(
        Box((0.008, 0.008, 0.002)),
        origin=Origin(xyz=(-0.037, -0.011, 0.0125)),
        material=power_blue,
        name="status_led",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((CONTROL_PANEL_SIZE[0], CONTROL_PANEL_SIZE[1], 0.016)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    lid_geom = section_loft(
        [
            _rounded_section(LID_SIZE[0], LID_SIZE[1], -0.001, 0.020, center_x=LID_CENTER_X, center_y=0.146),
            _rounded_section(LID_SIZE[0], LID_SIZE[1], 0.007, 0.020, center_x=LID_CENTER_X, center_y=0.146),
            _rounded_section(0.336, 0.270, 0.023, 0.017, center_x=LID_CENTER_X, center_y=0.142),
            _rounded_section(0.330, 0.264, 0.029, 0.015, center_x=LID_CENTER_X, center_y=0.138),
        ]
    )
    lid_mesh = _mesh("scanner_lid.obj", lid_geom)

    lid = model.part("lid")
    lid.visual(lid_mesh, material=warm_white)
    lid.visual(
        Cylinder(radius=0.008, length=0.124),
        origin=Origin(xyz=(0.0, 0.0, 0.0005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.302, 0.240, 0.008)),
        origin=Origin(xyz=(LID_CENTER_X, 0.148, 0.0085)),
        material=light_liner,
        name="lid_platen",
    )
    for side_x in (LID_CENTER_X - 0.128, LID_CENTER_X + 0.128):
        for side_y in (0.060, 0.244):
            lid.visual(
                Box((0.014, 0.014, 0.004)),
                origin=Origin(xyz=(side_x, side_y, 0.005)),
                material=rubber,
                name=f"lid_bumper_{'l' if side_x < 0.0 else 'r'}_{'rear' if side_y < 0.10 else 'front'}",
            )
    lid.inertial = Inertial.from_geometry(
        Box((LID_SIZE[0], LID_SIZE[1], 0.030)),
        mass=0.78,
        origin=Origin(xyz=(LID_CENTER_X, 0.146, 0.015)),
    )

    model.articulation(
        "base_to_scan_glass",
        ArticulationType.FIXED,
        parent="base",
        child="scan_glass",
        origin=Origin(xyz=(GLASS_CENTER[0], GLASS_CENTER[1], BODY_HEIGHT - 0.0035)),
    )
    model.articulation(
        "base_to_control_panel",
        ArticulationType.FIXED,
        parent="base",
        child="control_panel",
        origin=Origin(xyz=(CONTROL_PANEL_CENTER[0], CONTROL_PANEL_CENTER[1], BODY_HEIGHT - 0.003)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lid",
        origin=Origin(xyz=(0.0, -0.147, BODY_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("scan_glass", "base")
    ctx.expect_aabb_contact("control_panel", "base")
    ctx.expect_aabb_overlap("scan_glass", "base", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_gap("scan_glass", "base", axis="z", max_gap=0.003, max_penetration=0.016)

    ctx.expect_aabb_contact("lid", "scan_glass")
    ctx.expect_aabb_overlap("lid", "scan_glass", axes="xy", min_overlap=0.16)
    ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.10)
    ctx.expect_joint_motion_axis("lid_hinge", "lid", world_axis="z", direction="positive", min_delta=0.060)

    panel_pos = ctx.part_world_position("control_panel")
    glass_pos = ctx.part_world_position("scan_glass")
    if not (panel_pos[0] > glass_pos[0] + 0.14):
        raise AssertionError("Control area should sit clearly to the right of the scan bed.")
    if not (panel_pos[1] > glass_pos[1] + 0.08):
        raise AssertionError("Control area should sit toward the front of the scanner.")
    if not (BODY_HEIGHT - 0.006 <= glass_pos[2] <= BODY_HEIGHT - 0.002):
        raise AssertionError("Scan bed should sit near the scanner's top deck.")

    with ctx.pose(lid_hinge=1.25):
        ctx.expect_aabb_overlap("lid", "base", axes="x", min_overlap=0.30)
        ctx.expect_origin_distance("lid", "base", axes="x", max_dist=0.020)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
