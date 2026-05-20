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
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

GATE_WIDTH = 1.52
GATE_HEIGHT = 0.54
GATE_THICK = 0.036
SIDE_GAP = 0.012
OPENING_WIDTH = GATE_WIDTH + 2.0 * SIDE_GAP
STUB_WIDTH = 0.085
STUB_LENGTH = 0.18
STUB_CENTER_X = OPENING_WIDTH / 2.0 + STUB_WIDTH / 2.0
OVERALL_WIDTH = OPENING_WIDTH + 2.0 * STUB_WIDTH
HINGE_Z = 0.055
BED_TOP_Z = HINGE_Z + GATE_HEIGHT
REAR_SILL_DEPTH = 0.16
REAR_SILL_HEIGHT = 0.095
HINGE_BARREL_LENGTH = 0.065
HINGE_BARREL_RADIUS = 0.020
HINGE_BARREL_X = GATE_WIDTH / 2.0 - 0.0325
HINGE_MOUNT_LENGTH = 0.055
HINGE_MOUNT_X = HINGE_BARREL_X + (HINGE_BARREL_LENGTH + HINGE_MOUNT_LENGTH) / 2.0
PANEL_CORNER_RADIUS = 0.028
LATCH_CORNER_RADIUS = 0.012


def _box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _hinge_barrel(part, *, name: str, x: float, material) -> None:
    part.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(xyz=(x, -0.021, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _rounded_panel_mesh(
    *,
    filename: str,
    width: float,
    height: float,
    thickness: float,
    radius: float,
):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, radius=radius, corner_segments=10),
        thickness,
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate", assets=ASSETS)

    body_paint = model.material("body_paint", rgba=(0.69, 0.14, 0.16, 1.0))
    trim_black = model.material("trim_black", rgba=(0.13, 0.13, 0.13, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.26, 0.28, 0.30, 1.0))

    bed_frame = model.part("bed_frame")
    _box(
        bed_frame,
        name="rear_sill",
        size=(OVERALL_WIDTH, REAR_SILL_DEPTH, REAR_SILL_HEIGHT),
        xyz=(0.0, REAR_SILL_DEPTH / 2.0, REAR_SILL_HEIGHT / 2.0),
        material=liner_gray,
    )
    _box(
        bed_frame,
        name="left_stub",
        size=(STUB_WIDTH, STUB_LENGTH, BED_TOP_Z),
        xyz=(-STUB_CENTER_X, STUB_LENGTH / 2.0, BED_TOP_Z / 2.0),
        material=body_paint,
    )
    _box(
        bed_frame,
        name="right_stub",
        size=(STUB_WIDTH, STUB_LENGTH, BED_TOP_Z),
        xyz=(STUB_CENTER_X, STUB_LENGTH / 2.0, BED_TOP_Z / 2.0),
        material=body_paint,
    )
    _box(
        bed_frame,
        name="left_bedrail_cap",
        size=(STUB_WIDTH + 0.010, STUB_LENGTH * 0.95, 0.030),
        xyz=(-STUB_CENTER_X, STUB_LENGTH / 2.0, BED_TOP_Z - 0.015),
        material=trim_black,
    )
    _box(
        bed_frame,
        name="right_bedrail_cap",
        size=(STUB_WIDTH + 0.010, STUB_LENGTH * 0.95, 0.030),
        xyz=(STUB_CENTER_X, STUB_LENGTH / 2.0, BED_TOP_Z - 0.015),
        material=trim_black,
    )
    _box(
        bed_frame,
        name="left_jamb",
        size=(0.012, 0.030, 0.340),
        xyz=(-(OPENING_WIDTH / 2.0 + 0.006), 0.012, 0.300),
        material=trim_black,
    )
    _box(
        bed_frame,
        name="right_jamb",
        size=(0.012, 0.030, 0.340),
        xyz=(OPENING_WIDTH / 2.0 + 0.006, 0.012, 0.300),
        material=trim_black,
    )
    _box(
        bed_frame,
        name="left_hinge_mount",
        size=(HINGE_MOUNT_LENGTH, 0.055, 0.060),
        xyz=(-HINGE_MOUNT_X, -0.0025, HINGE_Z),
        material=trim_black,
    )
    _box(
        bed_frame,
        name="right_hinge_mount",
        size=(HINGE_MOUNT_LENGTH, 0.055, 0.060),
        xyz=(HINGE_MOUNT_X, -0.0025, HINGE_Z),
        material=trim_black,
    )
    bed_frame.inertial = Inertial.from_geometry(
        Box((OVERALL_WIDTH, STUB_LENGTH, BED_TOP_Z)),
        mass=42.0,
        origin=Origin(xyz=(0.0, STUB_LENGTH / 2.0, BED_TOP_Z / 2.0)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        _rounded_panel_mesh(
            filename="tailgate_panel.obj",
            width=GATE_WIDTH,
            height=0.520,
            thickness=GATE_THICK,
            radius=PANEL_CORNER_RADIUS,
        ),
        origin=Origin(xyz=(0.0, -0.020, 0.280)),
        material=body_paint,
        name="tailgate_panel",
    )
    _box(
        tailgate,
        name="bottom_hem",
        size=(GATE_WIDTH * 0.985, 0.038, 0.040),
        xyz=(0.0, -0.021, 0.020),
        material=body_paint,
    )
    _box(
        tailgate,
        name="inner_recess",
        size=(1.220, 0.014, 0.270),
        xyz=(0.0, 0.005, 0.265),
        material=liner_gray,
    )
    tailgate.visual(
        _rounded_panel_mesh(
            filename="tailgate_latch_band.obj",
            width=1.460,
            height=0.055,
            thickness=0.022,
            radius=LATCH_CORNER_RADIUS,
        ),
        origin=Origin(xyz=(0.0, 0.009, 0.5125)),
        material=trim_black,
        name="latch_band",
    )
    _hinge_barrel(tailgate, name="left_hinge_barrel", x=-HINGE_BARREL_X, material=trim_black)
    _hinge_barrel(tailgate, name="right_hinge_barrel", x=HINGE_BARREL_X, material=trim_black)
    tailgate.inertial = Inertial.from_geometry(
        Box((GATE_WIDTH, GATE_THICK, GATE_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.020, GATE_HEIGHT / 2.0)),
    )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bed_frame = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    tailgate_hinge = object_model.get_articulation("tailgate_hinge")

    rear_sill = bed_frame.get_visual("rear_sill")
    left_stub = bed_frame.get_visual("left_stub")
    left_bedrail_cap = bed_frame.get_visual("left_bedrail_cap")
    left_jamb = bed_frame.get_visual("left_jamb")
    right_jamb = bed_frame.get_visual("right_jamb")
    left_hinge_mount = bed_frame.get_visual("left_hinge_mount")
    right_hinge_mount = bed_frame.get_visual("right_hinge_mount")

    tailgate_panel = tailgate.get_visual("tailgate_panel")
    latch_band = tailgate.get_visual("latch_band")
    left_hinge_barrel = tailgate.get_visual("left_hinge_barrel")
    right_hinge_barrel = tailgate.get_visual("right_hinge_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        tailgate,
        bed_frame,
        elem_a=left_hinge_barrel,
        elem_b=left_hinge_mount,
        name="left_hinge_barrel_contacts_mount",
    )
    ctx.expect_contact(
        tailgate,
        bed_frame,
        elem_a=right_hinge_barrel,
        elem_b=right_hinge_mount,
        name="right_hinge_barrel_contacts_mount",
    )
    ctx.expect_gap(
        tailgate,
        bed_frame,
        axis="x",
        positive_elem=tailgate_panel,
        negative_elem=left_jamb,
        min_gap=0.008,
        max_gap=0.020,
        name="left_side_clearance",
    )
    ctx.expect_gap(
        bed_frame,
        tailgate,
        axis="x",
        positive_elem=right_jamb,
        negative_elem=tailgate_panel,
        min_gap=0.008,
        max_gap=0.020,
        name="right_side_clearance",
    )
    ctx.expect_overlap(
        tailgate,
        bed_frame,
        axes="x",
        min_overlap=1.40,
        elem_a=latch_band,
        elem_b=rear_sill,
        name="tailgate_spans_between_stubs",
    )

    left_stub_aabb = ctx.part_element_world_aabb(bed_frame, elem=left_stub)
    latch_aabb = ctx.part_element_world_aabb(tailgate, elem=latch_band)
    if left_stub_aabb is None or latch_aabb is None:
        ctx.fail("tailgate_closed_pose_measurements_available", "missing closed-pose AABB data")
    else:
        top_delta = abs(left_stub_aabb[1][2] - latch_aabb[1][2])
        ctx.check(
            "latch_band_aligns_with_bedrail_height",
            top_delta <= 0.020,
            details=f"stub top and latch band top differ by {top_delta:.4f} m",
        )

    with ctx.pose({tailgate_hinge: math.pi / 2.0}):
        ctx.expect_contact(
            tailgate,
            bed_frame,
            elem_a=left_hinge_barrel,
            elem_b=left_hinge_mount,
            name="left_hinge_stays_seated_open",
        )
        ctx.expect_contact(
            tailgate,
            bed_frame,
            elem_a=right_hinge_barrel,
            elem_b=right_hinge_mount,
            name="right_hinge_stays_seated_open",
        )
        ctx.expect_gap(
            bed_frame,
            tailgate,
            axis="y",
            positive_elem=rear_sill,
            negative_elem=tailgate_panel,
            min_gap=0.015,
            name="open_tailgate_swings_rearward_of_sill",
        )
        ctx.expect_overlap(
            tailgate,
            bed_frame,
            axes="x",
            min_overlap=1.40,
            elem_a=tailgate_panel,
            elem_b=rear_sill,
            name="open_tailgate_keeps_transverse_hinge_alignment",
        )

        open_latch_aabb = ctx.part_element_world_aabb(tailgate, elem=latch_band)
        open_cap_aabb = ctx.part_element_world_aabb(bed_frame, elem=left_bedrail_cap)
        if open_latch_aabb is None or open_cap_aabb is None:
            ctx.fail("tailgate_open_pose_measurements_available", "missing open-pose AABB data")
        else:
            ctx.check(
                "open_tailgate_lies_below_bedrail_caps",
                open_latch_aabb[1][2] < open_cap_aabb[1][2] - 0.45,
                details=(
                    f"open latch band max z={open_latch_aabb[1][2]:.4f} m, "
                    f"bedrail top z={open_cap_aabb[1][2]:.4f} m"
                ),
            )
            ctx.check(
                "open_tailgate_reaches_roughly_horizontal",
                open_latch_aabb[1][1] <= -0.45,
                details=f"open latch band max y={open_latch_aabb[1][1]:.4f} m",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
