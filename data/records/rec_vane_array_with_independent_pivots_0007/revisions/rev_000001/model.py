from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

N_VANES = 6
VANE_PITCH = 0.045
VANE_ZS = [((i - (N_VANES - 1) / 2.0) * VANE_PITCH) for i in range(N_VANES)]

PLATE_T = 0.012
HALF_CLEAR_SPAN = 0.128
PLATE_CENTER_X = HALF_CLEAR_SPAN + PLATE_T / 2.0
FRAME_OUTER_X = PLATE_CENTER_X + PLATE_T / 2.0

FRAME_DEPTH = 0.160
FRAME_HEIGHT = 0.350
WINDOW_DEPTH = 0.090
WINDOW_HEIGHT = 0.305

SPACER_SPAN = 2.0 * (HALF_CLEAR_SPAN + PLATE_T / 2.0)
SPACER_Y = 0.055
SPACER_Z = 0.150
SPACER_BAR_DEPTH = 0.020
SPACER_BAR_HEIGHT = 0.020

BEARING_BORE_R = 0.0064
OUTER_BOSS_R = 0.015
OUTER_BOSS_T = 0.006
INNER_BOSS_R = 0.011
INNER_BOSS_T = 0.004

STOP_TAB_Y = 0.032
STOP_TAB_OFFSET = 0.012
STOP_TAB_DEPTH = 0.012
STOP_TAB_HEIGHT = 0.010
PILLOW_BLOCK_X = 0.028
PILLOW_BLOCK_Y = 0.034
PILLOW_BLOCK_Z = 0.024
PILLOW_BLOCK_CENTER_X = 0.122
PILLOW_BORE_R = 0.0068
LANDING_RAIL_T = 0.006
SEAT_STRIP_Y = 0.014
SEAT_STRIP_Z = 0.014
POST_Y = 0.050
POST_DEPTH = 0.012
POST_WIDTH = 0.018
SUPPORT_PAD_X = 0.018
SUPPORT_PAD_Y = 0.010
SUPPORT_PAD_Z = 0.012
SUPPORT_PAD_Y_CENTER = 0.013

COVER_T = 0.006
COVER_DEPTH = 0.130
COVER_HEIGHT = 0.315
COVER_CAP_R = 0.013
COVER_CAP_T = 0.004
COVER_BOLT_R = 0.004
COVER_BOLT_T = 0.003

BLADE_HALF_SPAN = 0.114
BLADE_CHORD = 0.064
BLADE_THICKNESS = 0.013
JOURNAL_R = 0.0054
JOURNAL_LEN = 0.036
RETAINER_R = 0.009
RETAINER_T = 0.004
STUB_T = 0.006
STUB_REACH = 0.032
STUB_HEIGHT = 0.012
STUB_END_R = 0.008
STUB_PIN_HOLE_R = 0.0032

VANE_LIMIT = 0.78


def _frame_plate(is_left: bool) -> cq.Workplane:
    plate_sketch = cq.Sketch().rect(FRAME_DEPTH, FRAME_HEIGHT)
    plate_sketch = plate_sketch.rect(WINDOW_DEPTH, WINDOW_HEIGHT, mode="s")
    plate_sketch = plate_sketch.push([(0.0, z) for z in VANE_ZS]).circle(BEARING_BORE_R, mode="s")
    if is_left:
        tab_points = []
        for z in VANE_ZS:
            tab_points.extend(
                [
                    (STOP_TAB_Y, z + STOP_TAB_OFFSET),
                    (STOP_TAB_Y, z - STOP_TAB_OFFSET),
                ]
            )
        plate_sketch = plate_sketch.push(tab_points).rect(STOP_TAB_DEPTH, STOP_TAB_HEIGHT)

    plate = (
        cq.Workplane("YZ")
        .placeSketch(plate_sketch)
        .extrude(PLATE_T)
        .translate((-PLATE_T / 2.0, 0.0, 0.0))
    )

    outer_offset = PLATE_T / 2.0 if not is_left else -PLATE_T / 2.0 - OUTER_BOSS_T
    inner_offset = PLATE_T / 2.0 if is_left else -PLATE_T / 2.0 - INNER_BOSS_T

    outer_bosses = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, z) for z in VANE_ZS])
        .circle(OUTER_BOSS_R)
        .extrude(OUTER_BOSS_T)
        .translate((outer_offset, 0.0, 0.0))
    )
    inner_bosses = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, z) for z in VANE_ZS])
        .circle(INNER_BOSS_R)
        .extrude(INNER_BOSS_T)
        .translate((inner_offset, 0.0, 0.0))
    )

    hole_length = PLATE_T + OUTER_BOSS_T + INNER_BOSS_T + 0.006
    journal_holes = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, z) for z in VANE_ZS])
        .circle(BEARING_BORE_R)
        .extrude(hole_length)
        .translate((-hole_length / 2.0, 0.0, 0.0))
    )

    plate = plate.union(outer_bosses).union(inner_bosses).cut(journal_holes)

    gusset_centers = [
        (0.0, sy * 0.053, sz * 0.134)
        for sy in (-1.0, 1.0)
        for sz in (-1.0, 1.0)
    ]
    for center in gusset_centers:
        plate = plate.union(
            cq.Workplane("XY").box(0.020, 0.026, 0.040).translate(center)
        )

    x_pos = -PLATE_CENTER_X if is_left else PLATE_CENTER_X
    return plate.translate((x_pos, 0.0, 0.0))


def _frame_weldment() -> cq.Workplane:
    frame = cq.Workplane("XY").box(0.001, 0.001, 0.001)

    for sign in (-1.0, 1.0):
        frame = frame.union(
            cq.Workplane("XY")
            .box(LANDING_RAIL_T, COVER_DEPTH, COVER_HEIGHT)
            .translate((sign * (FRAME_OUTER_X - LANDING_RAIL_T / 2.0), 0.0, 0.0))
        )
        frame = frame.union(
            cq.Workplane("XY")
            .box(0.020, 0.026, FRAME_HEIGHT - 0.030)
            .translate((sign * 0.096, 0.0, 0.0))
        )

    for y in (-SPACER_Y, SPACER_Y):
        for z in (-SPACER_Z, SPACER_Z):
            frame = frame.union(
                cq.Workplane("XY")
                .box(SPACER_SPAN, SPACER_BAR_DEPTH, SPACER_BAR_HEIGHT)
                .translate((0.0, y, z))
            )

    for sign in (-1.0, 1.0):
        for vane_z in VANE_ZS:
            pillow = (
                cq.Workplane("XY")
                .box(PILLOW_BLOCK_X, PILLOW_BLOCK_Y, PILLOW_BLOCK_Z)
                .translate((sign * PILLOW_BLOCK_CENTER_X, 0.0, vane_z))
            )
            pillow_bore = (
                cq.Workplane("YZ")
                .circle(PILLOW_BORE_R)
                .extrude(PILLOW_BLOCK_X + 0.010)
                .translate(
                    (
                        sign * (PILLOW_BLOCK_CENTER_X - (PILLOW_BLOCK_X + 0.010) / 2.0),
                        0.0,
                        vane_z,
                    )
                )
            )
            frame = frame.union(pillow.cut(pillow_bore))

    stop_tabs = [
        (-0.110, STOP_TAB_Y, vane_z + STOP_TAB_OFFSET)
        for vane_z in VANE_ZS
    ] + [
        (-0.110, STOP_TAB_Y, vane_z - STOP_TAB_OFFSET)
        for vane_z in VANE_ZS
    ]
    for center in stop_tabs:
        frame = frame.union(
            cq.Workplane("XY").box(0.020, STOP_TAB_DEPTH, STOP_TAB_HEIGHT).translate(center)
        )

    return frame


def _cover_mesh(is_left: bool) -> cq.Workplane:
    cover_plate = (
        cq.Workplane("YZ")
        .placeSketch(cq.Sketch().rect(COVER_DEPTH, COVER_HEIGHT))
        .extrude(COVER_T)
        .translate((-COVER_T / 2.0, 0.0, 0.0))
    )

    cap_offset = COVER_T / 2.0 if not is_left else -COVER_T / 2.0 - COVER_CAP_T
    bolt_offset = COVER_T / 2.0 + COVER_CAP_T if not is_left else -COVER_T / 2.0 - COVER_CAP_T - COVER_BOLT_T

    cap_bosses = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, z) for z in VANE_ZS])
        .circle(COVER_CAP_R)
        .extrude(COVER_CAP_T)
        .translate((cap_offset, 0.0, 0.0))
    )

    bolt_points = [
        (-0.050, 0.140),
        (0.050, 0.140),
        (-0.050, 0.060),
        (0.050, 0.060),
        (-0.050, -0.060),
        (0.050, -0.060),
        (-0.050, -0.140),
        (0.050, -0.140),
    ]
    bolt_heads = (
        cq.Workplane("YZ")
        .pushPoints(bolt_points)
        .circle(COVER_BOLT_R)
        .extrude(COVER_BOLT_T)
        .translate((bolt_offset, 0.0, 0.0))
    )

    hole_length = COVER_T + COVER_CAP_T + 0.008
    journal_holes = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, z) for z in VANE_ZS])
        .circle(BEARING_BORE_R + 0.0004)
        .extrude(hole_length)
        .translate((-hole_length / 2.0, 0.0, 0.0))
    )

    return cover_plate.union(cap_bosses).union(bolt_heads).cut(journal_holes)


def _blade_mesh() -> cq.Workplane:
    airfoil = (
        cq.Workplane("YZ")
        .moveTo(-BLADE_CHORD / 2.0 + 0.005, 0.0)
        .spline(
            [
                (-0.020, BLADE_THICKNESS / 2.0),
                (0.020, BLADE_THICKNESS * 0.24),
                (BLADE_CHORD / 2.0, 0.0),
            ]
        )
        .spline(
            [
                (0.020, -BLADE_THICKNESS * 0.24),
                (-0.020, -BLADE_THICKNESS / 2.0),
                (-BLADE_CHORD / 2.0 + 0.005, 0.0),
            ]
        )
        .close()
        .extrude(2.0 * BLADE_HALF_SPAN)
        .translate((-BLADE_HALF_SPAN, 0.0, 0.0))
    )

    reinforcing_strips = []
    for z in (-0.0028, 0.0028):
        reinforcing_strips.append(
            cq.Workplane("XY")
            .box(2.0 * BLADE_HALF_SPAN, BLADE_CHORD * 0.54, 0.0016)
            .translate((0.0, 0.0, z))
        )

    blade = airfoil
    for strip in reinforcing_strips:
        blade = blade.union(strip)
    return blade


def _stub_mesh() -> cq.Workplane:
    stub_sketch = cq.Sketch().circle(RETAINER_R)
    stub_sketch = stub_sketch.push([(STUB_REACH / 2.0, 0.0)]).rect(STUB_REACH, STUB_HEIGHT)
    stub_sketch = stub_sketch.push([(STUB_REACH, 0.0)]).circle(STUB_END_R)
    stub_sketch = stub_sketch.circle(JOURNAL_R + 0.0005, mode="s")
    stub_sketch = stub_sketch.push([(STUB_REACH, 0.0)]).circle(STUB_PIN_HOLE_R, mode="s")
    return (
        cq.Workplane("YZ")
        .placeSketch(stub_sketch)
        .extrude(STUB_T)
        .translate((-STUB_T / 2.0, 0.0, 0.0))
    )


def _retainer_mesh() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .placeSketch(cq.Sketch().circle(RETAINER_R).circle(JOURNAL_R + 0.0005, mode="s"))
        .extrude(RETAINER_T)
        .translate((-RETAINER_T / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vane_array_study", assets=ASSETS)

    frame_steel = model.material("frame_steel", rgba=(0.33, 0.35, 0.37, 1.0))
    cover_oxide = model.material("cover_oxide", rgba=(0.16, 0.17, 0.18, 1.0))
    vane_gray = model.material("vane_gray", rgba=(0.48, 0.49, 0.51, 1.0))
    hardware = model.material("hardware", rgba=(0.60, 0.61, 0.63, 1.0))

    left_cover_mesh = mesh_from_cadquery(_cover_mesh(is_left=True), "left_cover.obj", assets=ASSETS)
    right_cover_mesh = mesh_from_cadquery(_cover_mesh(is_left=False), "right_cover.obj", assets=ASSETS)

    frame = model.part("frame")
    for sign, tag in ((-1.0, "left"), (1.0, "right")):
        frame.visual(
            Box((LANDING_RAIL_T, COVER_DEPTH, SEAT_STRIP_Z)),
            origin=Origin(
                xyz=(sign * (FRAME_OUTER_X - LANDING_RAIL_T / 2.0), 0.0, SPACER_Z),
            ),
            material=frame_steel,
            name=f"{tag}_top_seat",
        )
        frame.visual(
            Box((LANDING_RAIL_T, COVER_DEPTH, SEAT_STRIP_Z)),
            origin=Origin(
                xyz=(sign * (FRAME_OUTER_X - LANDING_RAIL_T / 2.0), 0.0, -SPACER_Z),
            ),
            material=frame_steel,
            name=f"{tag}_bottom_seat",
        )
        frame.visual(
            Box((LANDING_RAIL_T, SEAT_STRIP_Y, COVER_HEIGHT - 2.0 * SEAT_STRIP_Z)),
            origin=Origin(
                xyz=(sign * (FRAME_OUTER_X - LANDING_RAIL_T / 2.0), POST_Y, 0.0),
            ),
            material=frame_steel,
            name=f"{tag}_front_seat",
        )
        frame.visual(
            Box((LANDING_RAIL_T, SEAT_STRIP_Y, COVER_HEIGHT - 2.0 * SEAT_STRIP_Z)),
            origin=Origin(
                xyz=(sign * (FRAME_OUTER_X - LANDING_RAIL_T / 2.0), -POST_Y, 0.0),
            ),
            material=frame_steel,
            name=f"{tag}_rear_seat",
        )
        frame.visual(
            Box((POST_WIDTH, POST_DEPTH, FRAME_HEIGHT - 0.030)),
            origin=Origin(xyz=(sign * 0.096, POST_Y, 0.0)),
            material=frame_steel,
            name=f"{tag}_front_post",
        )
        frame.visual(
            Box((POST_WIDTH, POST_DEPTH, FRAME_HEIGHT - 0.030)),
            origin=Origin(xyz=(sign * 0.096, -POST_Y, 0.0)),
            material=frame_steel,
            name=f"{tag}_rear_post",
        )

    for y_sign, y_tag in ((-1.0, "rear"), (1.0, "front")):
        for z_sign, z_tag in ((-1.0, "lower"), (1.0, "upper")):
            frame.visual(
                Box((SPACER_SPAN, SPACER_BAR_DEPTH, SPACER_BAR_HEIGHT)),
                origin=Origin(xyz=(0.0, y_sign * SPACER_Y, z_sign * SPACER_Z)),
                material=frame_steel,
                name=f"{y_tag}_{z_tag}_spacer",
            )

    for sign, tag in ((-1.0, "left"), (1.0, "right")):
        for index, vane_z in enumerate(VANE_ZS, start=1):
            frame.visual(
                Box((SUPPORT_PAD_X, SUPPORT_PAD_Y, SUPPORT_PAD_Z)),
                origin=Origin(
                    xyz=(sign * PILLOW_BLOCK_CENTER_X, SUPPORT_PAD_Y_CENTER, vane_z),
                ),
                material=frame_steel,
                name=f"{tag}_upper_support_{index}",
            )
            frame.visual(
                Box((SUPPORT_PAD_X, SUPPORT_PAD_Y, SUPPORT_PAD_Z)),
                origin=Origin(
                    xyz=(sign * PILLOW_BLOCK_CENTER_X, -SUPPORT_PAD_Y_CENTER, vane_z),
                ),
                material=frame_steel,
                name=f"{tag}_lower_support_{index}",
            )

    for index, vane_z in enumerate(VANE_ZS, start=1):
        frame.visual(
            Box((0.020, STOP_TAB_DEPTH, STOP_TAB_HEIGHT)),
            origin=Origin(xyz=(-0.110, STOP_TAB_Y, vane_z + STOP_TAB_OFFSET)),
            material=hardware,
            name=f"stop_tab_upper_{index}",
        )
        frame.visual(
            Box((0.020, STOP_TAB_DEPTH, STOP_TAB_HEIGHT)),
            origin=Origin(xyz=(-0.110, STOP_TAB_Y, vane_z - STOP_TAB_OFFSET)),
            material=hardware,
            name=f"stop_tab_lower_{index}",
        )
    frame.inertial = Inertial.from_geometry(
        Box((2.0 * FRAME_OUTER_X, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=18.0,
    )

    left_cover = model.part("left_cover")
    left_cover.visual(left_cover_mesh, material=cover_oxide, name="cover_shell")
    left_cover.inertial = Inertial.from_geometry(
        Box((COVER_T, COVER_DEPTH, COVER_HEIGHT)),
        mass=1.1,
    )

    right_cover = model.part("right_cover")
    right_cover.visual(right_cover_mesh, material=cover_oxide, name="cover_shell")
    right_cover.inertial = Inertial.from_geometry(
        Box((COVER_T, COVER_DEPTH, COVER_HEIGHT)),
        mass=1.1,
    )

    model.articulation(
        "frame_to_left_cover",
        ArticulationType.FIXED,
        parent=frame,
        child=left_cover,
        origin=Origin(xyz=(-(FRAME_OUTER_X + COVER_T / 2.0), 0.0, 0.0)),
    )
    model.articulation(
        "frame_to_right_cover",
        ArticulationType.FIXED,
        parent=frame,
        child=right_cover,
        origin=Origin(xyz=((FRAME_OUTER_X + COVER_T / 2.0), 0.0, 0.0)),
    )

    journal_center_x = BLADE_HALF_SPAN + JOURNAL_LEN / 2.0
    left_stub_center_x = -(FRAME_OUTER_X + COVER_T + 0.001 + STUB_T / 2.0)
    right_retainer_center_x = FRAME_OUTER_X + COVER_T + 0.001 + RETAINER_T / 2.0

    for index, vane_z in enumerate(VANE_ZS, start=1):
        vane = model.part(f"vane_{index}")
        vane.visual(
            Box((2.0 * BLADE_HALF_SPAN, BLADE_CHORD * 0.72, BLADE_THICKNESS * 0.55)),
            material=vane_gray,
            name="blade_shell",
        )
        vane.visual(
            Cylinder(radius=BLADE_THICKNESS * 0.5, length=2.0 * BLADE_HALF_SPAN),
            origin=Origin(
                xyz=(0.0, -BLADE_CHORD * 0.23, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=vane_gray,
            name="leading_edge",
        )
        vane.visual(
            Box((2.0 * BLADE_HALF_SPAN, BLADE_CHORD * 0.18, BLADE_THICKNESS * 0.22)),
            origin=Origin(xyz=(0.0, BLADE_CHORD * 0.29, 0.0)),
            material=vane_gray,
            name="trailing_strip",
        )
        vane.visual(
            Cylinder(radius=JOURNAL_R, length=JOURNAL_LEN),
            origin=Origin(xyz=(-journal_center_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="left_journal",
        )
        vane.visual(
            Cylinder(radius=JOURNAL_R, length=JOURNAL_LEN),
            origin=Origin(xyz=(journal_center_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="right_journal",
        )
        vane.visual(
            Box((STUB_T, STUB_REACH, STUB_HEIGHT)),
            origin=Origin(xyz=(left_stub_center_x, STUB_REACH / 2.0, 0.0)),
            material=hardware,
            name="left_stub",
        )
        vane.visual(
            Cylinder(radius=RETAINER_R, length=RETAINER_T),
            origin=Origin(
                xyz=(right_retainer_center_x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hardware,
            name="right_retainer",
        )
        vane.inertial = Inertial.from_geometry(
            Box((2.0 * (FRAME_OUTER_X + COVER_T + RETAINER_T), BLADE_CHORD, 0.024)),
            mass=0.62,
        )

        model.articulation(
            f"frame_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, vane_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=-VANE_LIMIT,
                upper=VANE_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    left_cover = object_model.get_part("left_cover")
    right_cover = object_model.get_part("right_cover")
    left_cover_shell = left_cover.get_visual("cover_shell")
    right_cover_shell = right_cover.get_visual("cover_shell")

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

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(left_cover, frame, name="left_cover_contacts_frame")
    ctx.expect_contact(right_cover, frame, name="right_cover_contacts_frame")
    ctx.expect_overlap(left_cover, frame, axes="yz", min_overlap=0.10, name="left_cover_overlaps_frame_footprint")
    ctx.expect_overlap(right_cover, frame, axes="yz", min_overlap=0.10, name="right_cover_overlaps_frame_footprint")

    ctx.expect_gap(
        frame,
        left_cover,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        name="left_cover_seats_on_frame",
    )
    ctx.expect_gap(
        right_cover,
        frame,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        name="right_cover_seats_on_frame",
    )

    representative_poses = {2: -0.52, 3: 0.34, 4: 0.52, 5: -0.34}

    for index in range(1, N_VANES + 1):
        vane = object_model.get_part(f"vane_{index}")
        articulation = object_model.get_articulation(f"frame_to_vane_{index}")
        left_journal = vane.get_visual("left_journal")
        right_journal = vane.get_visual("right_journal")
        left_stub = vane.get_visual("left_stub")
        right_retainer = vane.get_visual("right_retainer")

        ctx.check(f"vane_{index}_present", vane is not None, f"Missing vane_{index}")
        ctx.check(
            f"vane_{index}_axis_is_x",
            tuple(round(v, 6) for v in articulation.axis) == (1.0, 0.0, 0.0),
            f"Unexpected axis for vane_{index}: {articulation.axis}",
        )
        ctx.check(
            f"vane_{index}_limits_match_stop_tabs",
            abs(articulation.motion_limits.lower + VANE_LIMIT) < 1e-9
            and abs(articulation.motion_limits.upper - VANE_LIMIT) < 1e-9,
            f"Unexpected limits for vane_{index}: {articulation.motion_limits}",
        )

        ctx.expect_origin_distance(
            vane,
            frame,
            axes="xy",
            max_dist=1e-6,
            name=f"vane_{index}_pivot_stays_on_frame_centerplane",
        )
        ctx.expect_overlap(
            vane,
            left_cover,
            axes="yz",
            min_overlap=0.010,
            elem_a=left_journal,
            elem_b=left_cover_shell,
            name=f"vane_{index}_left_journal_registers_with_left_cover",
        )
        ctx.expect_overlap(
            vane,
            right_cover,
            axes="yz",
            min_overlap=0.010,
            elem_a=right_journal,
            elem_b=right_cover_shell,
            name=f"vane_{index}_right_journal_registers_with_right_cover",
        )
        ctx.expect_gap(
            left_cover,
            vane,
            axis="x",
            min_gap=0.0005,
            max_gap=0.0020,
            positive_elem=left_cover_shell,
            negative_elem=left_stub,
            name=f"vane_{index}_left_stub_runs_outboard_of_cover",
        )
        ctx.expect_gap(
            vane,
            right_cover,
            axis="x",
            min_gap=0.0005,
            max_gap=0.0020,
            positive_elem=right_retainer,
            negative_elem=right_cover_shell,
            name=f"vane_{index}_right_retainer_runs_outboard_of_cover",
        )

        commanded = representative_poses.get(index, 0.22 if index % 2 else -0.22)
        with ctx.pose({articulation: commanded}):
            ctx.expect_overlap(
                vane,
                left_cover,
                axes="yz",
                min_overlap=0.010,
                elem_a=left_journal,
                elem_b=left_cover_shell,
                name=f"vane_{index}_left_support_stays_registered_in_pose",
            )
            ctx.expect_overlap(
                vane,
                right_cover,
                axes="yz",
                min_overlap=0.010,
                elem_a=right_journal,
                elem_b=right_cover_shell,
                name=f"vane_{index}_right_support_stays_registered_in_pose",
            )
            ctx.expect_gap(
                left_cover,
                vane,
                axis="x",
                min_gap=0.0005,
                max_gap=0.0020,
                positive_elem=left_cover_shell,
                negative_elem=left_stub,
                name=f"vane_{index}_left_stub_clearance_in_pose",
            )
            ctx.expect_gap(
                vane,
                right_cover,
                axis="x",
                min_gap=0.0005,
                max_gap=0.0020,
                positive_elem=right_retainer,
                negative_elem=right_cover_shell,
                name=f"vane_{index}_right_retainer_clearance_in_pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
