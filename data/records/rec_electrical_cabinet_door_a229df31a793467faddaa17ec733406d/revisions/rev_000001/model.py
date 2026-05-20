from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="meter_cubicle")

    painted_steel = model.material("painted_steel", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.28, 0.31, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.12, 0.12, 0.13, 1.0))
    breaker_black = model.material("breaker_black", rgba=(0.16, 0.16, 0.18, 1.0))
    meter_white = model.material("meter_white", rgba=(0.92, 0.93, 0.95, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.67, 0.79, 0.88, 0.28))
    backboard_beige = model.material("backboard_beige", rgba=(0.82, 0.77, 0.67, 1.0))

    cabinet_w = 0.72
    cabinet_d = 0.26
    cabinet_h = 1.10
    shell_t = 0.02
    front_face_t = 0.02

    opening_w = 0.46
    opening_h = 0.60
    opening_bottom_z = 0.26
    opening_top_z = opening_bottom_z + opening_h

    cover_w = 0.54
    cover_h = 0.69
    cover_t = 0.03
    frame_w = 0.045
    pane_capture = 0.01
    hinge_radius = 0.014
    hinge_y = cabinet_d / 2.0 + 0.002 + cover_t / 2.0

    enclosure = model.part("enclosure")
    enclosure.visual(
        Box((cabinet_w, shell_t, cabinet_h)),
        origin=Origin(xyz=(0.0, -cabinet_d / 2.0 + shell_t / 2.0, cabinet_h / 2.0)),
        material=painted_steel,
        name="back_panel",
    )
    enclosure.visual(
        Box((shell_t, cabinet_d, cabinet_h)),
        origin=Origin(xyz=(-cabinet_w / 2.0 + shell_t / 2.0, 0.0, cabinet_h / 2.0)),
        material=painted_steel,
        name="left_wall",
    )
    enclosure.visual(
        Box((shell_t, cabinet_d, cabinet_h)),
        origin=Origin(xyz=(cabinet_w / 2.0 - shell_t / 2.0, 0.0, cabinet_h / 2.0)),
        material=painted_steel,
        name="right_wall",
    )
    enclosure.visual(
        Box((cabinet_w, cabinet_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t / 2.0)),
        material=painted_steel,
        name="bottom_panel",
    )
    enclosure.visual(
        Box((cabinet_w, cabinet_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_h - shell_t / 2.0)),
        material=painted_steel,
        name="top_panel",
    )
    enclosure.visual(
        Box((cabinet_w, front_face_t, cabinet_h - opening_top_z)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_d / 2.0 - front_face_t / 2.0,
                opening_top_z + (cabinet_h - opening_top_z) / 2.0,
            )
        ),
        material=painted_steel,
        name="front_top_plate",
    )
    enclosure.visual(
        Box((cabinet_w, front_face_t, opening_bottom_z)),
        origin=Origin(
            xyz=(0.0, cabinet_d / 2.0 - front_face_t / 2.0, opening_bottom_z / 2.0)
        ),
        material=painted_steel,
        name="front_bottom_plate",
    )
    side_plate_w = (cabinet_w - opening_w) / 2.0
    enclosure.visual(
        Box((side_plate_w, front_face_t, opening_h)),
        origin=Origin(
            xyz=(
                -opening_w / 2.0 - side_plate_w / 2.0,
                cabinet_d / 2.0 - front_face_t / 2.0,
                opening_bottom_z + opening_h / 2.0,
            )
        ),
        material=painted_steel,
        name="front_left_plate",
    )
    enclosure.visual(
        Box((side_plate_w, front_face_t, opening_h)),
        origin=Origin(
            xyz=(
                opening_w / 2.0 + side_plate_w / 2.0,
                cabinet_d / 2.0 - front_face_t / 2.0,
                opening_bottom_z + opening_h / 2.0,
            )
        ),
        material=painted_steel,
        name="front_right_plate",
    )
    enclosure.visual(
        Cylinder(radius=hinge_radius, length=0.10),
        origin=Origin(xyz=(-0.17, hinge_y, opening_top_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle_left",
    )
    enclosure.visual(
        Cylinder(radius=hinge_radius, length=0.10),
        origin=Origin(xyz=(0.17, hinge_y, opening_top_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle_right",
    )
    enclosure.visual(
        Box((0.09, 0.01, 0.036)),
        origin=Origin(xyz=(-0.17, cabinet_d / 2.0 + 0.005, opening_top_z + 0.018)),
        material=dark_steel,
        name="hinge_leaf_left",
    )
    enclosure.visual(
        Box((0.09, 0.01, 0.036)),
        origin=Origin(xyz=(0.17, cabinet_d / 2.0 + 0.005, opening_top_z + 0.018)),
        material=dark_steel,
        name="hinge_leaf_right",
    )
    enclosure.visual(
        Box((0.08, 0.006, 0.03)),
        origin=Origin(xyz=(0.0, cabinet_d / 2.0 - 0.003, 0.205)),
        material=dark_steel,
        name="latch_keeper",
    )
    enclosure.inertial = Inertial.from_geometry(
        Box((cabinet_w, cabinet_d, cabinet_h)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_h / 2.0)),
    )

    meter_bank = model.part("meter_bank")
    meter_bank.visual(
        Box((0.52, 0.018, 0.72)),
        origin=Origin(xyz=(0.0, -0.045, 0.56)),
        material=backboard_beige,
        name="meter_backboard",
    )
    meter_bank.visual(
        Box((0.46, 0.04, 0.60)),
        origin=Origin(xyz=(0.0, -0.025, 0.57)),
        material=dark_steel,
        name="meter_pan",
    )
    meter_bank.visual(
        Box((0.06, 0.058, 0.035)),
        origin=Origin(xyz=(-0.18, -0.083, 0.56)),
        material=dark_steel,
        name="mount_rail_left",
    )
    meter_bank.visual(
        Box((0.06, 0.058, 0.035)),
        origin=Origin(xyz=(0.18, -0.083, 0.56)),
        material=dark_steel,
        name="mount_rail_right",
    )

    meter_positions = (
        ("upper_left", -0.14, 0.70),
        ("upper_right", 0.14, 0.70),
        ("lower_left", -0.14, 0.47),
        ("lower_right", 0.14, 0.47),
    )
    for label, x_pos, z_pos in meter_positions:
        meter_bank.visual(
            Cylinder(radius=0.084, length=0.072),
            origin=Origin(xyz=(x_pos, 0.001, z_pos), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=breaker_black,
            name=f"meter_body_{label}",
        )
        meter_bank.visual(
            Cylinder(radius=0.066, length=0.012),
            origin=Origin(xyz=(x_pos, 0.039, z_pos), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=meter_white,
            name=f"meter_dial_{label}",
        )

    meter_bank.visual(
        Box((0.34, 0.09, 0.11)),
        origin=Origin(xyz=(0.0, -0.004, 0.305)),
        material=gasket_black,
        name="terminal_block",
    )
    meter_bank.visual(
        Box((0.18, 0.05, 0.13)),
        origin=Origin(xyz=(0.0, 0.016, 0.24)),
        material=breaker_black,
        name="breaker_stack",
    )
    meter_bank.inertial = Inertial.from_geometry(
        Box((0.52, 0.12, 0.74)),
        mass=14.0,
        origin=Origin(xyz=(0.0, -0.01, 0.55)),
    )

    cover_panel = model.part("cover_panel")
    cover_panel.visual(
        Box((cover_w, cover_t, frame_w)),
        origin=Origin(xyz=(0.0, 0.0, -(hinge_radius + frame_w / 2.0 + 0.0005))),
        material=painted_steel,
        name="top_rail",
    )
    cover_panel.visual(
        Box((cover_w, cover_t, frame_w)),
        origin=Origin(xyz=(0.0, 0.0, -cover_h + frame_w / 2.0)),
        material=painted_steel,
        name="bottom_rail",
    )
    cover_panel.visual(
        Box((frame_w, cover_t, cover_h)),
        origin=Origin(xyz=(-cover_w / 2.0 + frame_w / 2.0, 0.0, -cover_h / 2.0)),
        material=painted_steel,
        name="left_stile",
    )
    cover_panel.visual(
        Box((frame_w, cover_t, cover_h)),
        origin=Origin(xyz=(cover_w / 2.0 - frame_w / 2.0, 0.0, -cover_h / 2.0)),
        material=painted_steel,
        name="right_stile",
    )
    cover_panel.visual(
        Box(
            (
                cover_w - 2.0 * (frame_w - pane_capture),
                0.006,
                cover_h - 2.0 * (frame_w - pane_capture),
            )
        ),
        origin=Origin(xyz=(0.0, -0.006, -cover_h / 2.0)),
        material=smoked_glass,
        name="window_glass",
    )
    cover_panel.visual(
        Cylinder(radius=hinge_radius, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="center_hinge_knuckle",
    )
    cover_panel.visual(
        Box((0.22, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, -0.011, -0.013)),
        material=dark_steel,
        name="center_hinge_leaf",
    )
    cover_panel.visual(
        Cylinder(radius=0.017, length=0.022),
        origin=Origin(xyz=(0.0, 0.011, -0.662), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="latch_shaft",
    )
    cover_panel.visual(
        Box((0.05, 0.016, 0.06)),
        origin=Origin(xyz=(0.0, 0.025, -0.662)),
        material=dark_steel,
        name="bottom_latch_handle",
    )
    cover_panel.visual(
        Box((0.06, 0.01, 0.024)),
        origin=Origin(xyz=(0.0, -0.008, -0.662)),
        material=dark_steel,
        name="bottom_latch_cam",
    )
    cover_panel.inertial = Inertial.from_geometry(
        Box((cover_w, cover_t, cover_h)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -cover_h / 2.0)),
    )

    model.articulation(
        "enclosure_to_meter_bank",
        ArticulationType.FIXED,
        parent=enclosure,
        child=meter_bank,
    )
    model.articulation(
        "window_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=cover_panel,
        origin=Origin(xyz=(0.0, hinge_y, opening_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=1.25,
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
    enclosure = object_model.get_part("enclosure")
    meter_bank = object_model.get_part("meter_bank")
    cover_panel = object_model.get_part("cover_panel")
    hinge = object_model.get_articulation("window_cover_hinge")

    window_glass = cover_panel.get_visual("window_glass")
    bottom_rail = cover_panel.get_visual("bottom_rail")
    latch_cam = cover_panel.get_visual("bottom_latch_cam")
    keeper = enclosure.get_visual("latch_keeper")
    front_bottom_plate = enclosure.get_visual("front_bottom_plate")
    upper_left_dial = meter_bank.get_visual("meter_dial_upper_left")

    ctx.check(
        "hinge axis runs along the top edge",
        tuple(round(v, 6) for v in hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={hinge.axis}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            cover_panel,
            enclosure,
            axes="xz",
            min_overlap=0.45,
            name="closed cover spans the meter opening",
        )
        ctx.expect_gap(
            cover_panel,
            enclosure,
            axis="y",
            positive_elem=bottom_rail,
            negative_elem=front_bottom_plate,
            min_gap=0.001,
            max_gap=0.006,
            name="closed cover sits just proud of the enclosure face",
        )
        ctx.expect_gap(
            cover_panel,
            meter_bank,
            axis="y",
            positive_elem=window_glass,
            negative_elem=upper_left_dial,
            min_gap=0.08,
            name="glass clears the front of the meters",
        )
        ctx.expect_overlap(
            cover_panel,
            enclosure,
            axes="xz",
            elem_a=latch_cam,
            elem_b=keeper,
            min_overlap=0.018,
            name="bottom latch aligns over the keeper",
        )

    closed_bottom = ctx.part_element_world_aabb(cover_panel, elem="bottom_rail")
    with ctx.pose({hinge: 1.1}):
        ctx.expect_gap(
            cover_panel,
            enclosure,
            axis="y",
            positive_elem=bottom_rail,
            negative_elem=front_bottom_plate,
            min_gap=0.40,
            name="opened cover swings clear of the enclosure",
        )
        open_bottom = ctx.part_element_world_aabb(cover_panel, elem="bottom_rail")

    if closed_bottom is None or open_bottom is None:
        ctx.fail(
            "cover bottom edge can be measured",
            details=f"closed_bottom={closed_bottom}, open_bottom={open_bottom}",
        )
    else:
        closed_center = tuple(
            (closed_bottom[0][idx] + closed_bottom[1][idx]) / 2.0 for idx in range(3)
        )
        open_center = tuple(
            (open_bottom[0][idx] + open_bottom[1][idx]) / 2.0 for idx in range(3)
        )
        ctx.check(
            "bottom edge lifts and moves outward when opened",
            open_center[1] > closed_center[1] + 0.25
            and open_center[2] > closed_center[2] + 0.20,
            details=f"closed_center={closed_center}, open_center={open_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
