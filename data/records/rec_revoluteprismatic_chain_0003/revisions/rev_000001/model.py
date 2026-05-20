from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
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


def _export_mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(
        shape,
        filename,
        assets=ASSETS,
        tolerance=0.0007,
        angular_tolerance=0.08,
    )


def _frame_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.20, 0.11, 0.016).translate((0.040, 0.000, -0.058))
    front_foot = cq.Workplane("XY").box(0.044, 0.085, 0.014).translate((0.112, 0.000, -0.073))
    rear_foot = cq.Workplane("XY").box(0.040, 0.078, 0.014).translate((-0.040, 0.000, -0.073))
    pedestal = cq.Workplane("XY").box(0.052, 0.074, 0.020).translate((-0.004, 0.000, -0.040))
    left_cheek = cq.Workplane("XY").box(0.036, 0.012, 0.096).translate((0.000, 0.040, -0.002))
    right_cheek = cq.Workplane("XY").box(0.036, 0.012, 0.096).translate((0.000, -0.040, -0.002))
    bridge = cq.Workplane("XY").box(0.016, 0.064, 0.016).translate((-0.020, 0.000, 0.030))
    left_web = cq.Workplane("XY").box(0.032, 0.012, 0.042).translate((-0.008, 0.040, -0.029))
    right_web = cq.Workplane("XY").box(0.032, 0.012, 0.042).translate((-0.008, -0.040, -0.029))
    left_side_rib = cq.Workplane("XY").box(0.070, 0.010, 0.022).translate((0.050, 0.026, -0.036))
    right_side_rib = cq.Workplane("XY").box(0.070, 0.010, 0.022).translate((0.050, -0.026, -0.036))

    frame = (
        base.union(front_foot)
        .union(rear_foot)
        .union(pedestal)
        .union(left_cheek)
        .union(right_cheek)
        .union(bridge)
        .union(left_web)
        .union(right_web)
        .union(left_side_rib)
        .union(right_side_rib)
    )

    trunnion_clear = cq.Workplane("XZ").workplane(offset=-0.048).circle(0.0118).extrude(0.096)
    lightening_slot_1 = cq.Workplane("XY").box(0.050, 0.018, 0.024).translate((0.006, 0.000, -0.056))
    lightening_slot_2 = cq.Workplane("XY").box(0.045, 0.018, 0.024).translate((0.082, 0.000, -0.056))
    cable_slot = cq.Workplane("XY").box(0.026, 0.052, 0.016).translate((-0.020, 0.000, -0.040))
    mount_holes = (
        cq.Workplane("XY")
        .workplane(offset=-0.080)
        .pushPoints([(-0.040, 0.032), (-0.040, -0.032), (0.120, 0.034), (0.120, -0.034)])
        .circle(0.0045)
        .extrude(0.030)
    )

    return (
        frame.cut(trunnion_clear)
        .cut(lightening_slot_1)
        .cut(lightening_slot_2)
        .cut(cable_slot)
        .cut(mount_holes)
    )


def _swing_stage_shape() -> cq.Workplane:
    rod_radius = 0.0075

    hub = cq.Workplane("XZ").workplane(offset=-0.024).circle(0.022).extrude(0.048)
    shaft = cq.Workplane("XZ").workplane(offset=-0.046).circle(0.0110).extrude(0.092)
    left_collar = cq.Workplane("XZ").workplane(offset=-0.052).circle(0.020).extrude(0.006)
    right_collar = cq.Workplane("XZ").workplane(offset=0.046).circle(0.020).extrude(0.006)

    beam = cq.Workplane("XY").box(0.172, 0.044, 0.020).translate((0.114, 0.000, 0.024))
    guide_rail = cq.Workplane("XY").box(0.152, 0.018, 0.010).translate((0.114, 0.000, 0.050))
    rear_anchor = cq.Workplane("XY").box(0.020, 0.050, 0.034).translate((0.034, 0.000, 0.047))
    front_anchor = cq.Workplane("XY").box(0.016, 0.050, 0.034).translate((0.194, 0.000, 0.047))
    lower_rib = (
        cq.Workplane("XZ")
        .polyline([(-0.004, 0.004), (0.070, 0.004), (0.112, 0.030), (0.016, 0.032)])
        .close()
        .extrude(0.032, both=True)
    )
    front_stop = cq.Workplane("XY").box(0.010, 0.032, 0.018).translate((0.194, 0.000, 0.060))
    lockout_tab = cq.Workplane("XY").box(0.018, 0.004, 0.022).translate((0.024, -0.031, 0.050))
    side_land = cq.Workplane("XY").box(0.076, 0.006, 0.034).translate((0.096, 0.025, 0.016))

    stage = (
        hub.union(shaft)
        .union(left_collar)
        .union(right_collar)
        .union(beam)
        .union(guide_rail)
        .union(rear_anchor)
        .union(front_anchor)
        .union(lower_rib)
        .union(front_stop)
        .union(lockout_tab)
        .union(side_land)
    )

    left_rod_bore = cq.Workplane("YZ").workplane(offset=0.030).center(0.047, 0.022).circle(rod_radius).extrude(0.164)
    right_rod_bore = cq.Workplane("YZ").workplane(offset=0.030).center(0.047, -0.022).circle(rod_radius).extrude(0.164)
    service_pocket = cq.Workplane("XY").box(0.064, 0.010, 0.024).translate((0.096, 0.025, 0.016))
    top_relief = cq.Workplane("XY").box(0.094, 0.020, 0.010).translate((0.118, 0.000, 0.040))

    return stage.cut(left_rod_bore).cut(right_rod_bore).cut(service_pocket).cut(top_relief)


def _guide_rod_shape() -> cq.Workplane:
    return cq.Workplane("YZ").workplane(offset=-0.082).circle(0.0075).extrude(0.164)


def _access_cover_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.076, 0.003, 0.038)
    heads = (
        cq.Workplane("XZ")
        .workplane(offset=0.0015)
        .pushPoints([(-0.028, -0.012), (-0.028, 0.012), (0.028, -0.012), (0.028, 0.012)])
        .circle(0.0032)
        .extrude(0.0012)
    )
    center_slot = cq.Workplane("XY").box(0.036, 0.004, 0.010)
    return plate.union(heads).cut(center_slot)


def _carriage_shape() -> cq.Workplane:
    rod_outer = 0.0115
    rod_clear = 0.0082

    left_sleeve = cq.Workplane("YZ").workplane(offset=0.000).center(0.000, 0.022).circle(rod_outer).extrude(0.076)
    right_sleeve = cq.Workplane("YZ").workplane(offset=0.000).center(0.000, -0.022).circle(rod_outer).extrude(0.076)
    bridge = cq.Workplane("XY").box(0.056, 0.062, 0.016).translate((0.032, 0.000, 0.023))
    wear_pad = cq.Workplane("XY").box(0.048, 0.014, 0.006).translate((0.030, 0.000, 0.011))
    top_plate = cq.Workplane("XY").box(0.048, 0.050, 0.010).translate((0.044, 0.000, 0.040))
    left_cheek = cq.Workplane("XY").box(0.016, 0.012, 0.026).translate((0.076, 0.020, 0.034))
    right_cheek = cq.Workplane("XY").box(0.016, 0.012, 0.026).translate((0.076, -0.020, 0.034))
    lock_tab = cq.Workplane("XY").box(0.010, 0.026, 0.014).translate((0.010, 0.000, 0.036))

    carriage = (
        left_sleeve.union(right_sleeve)
        .union(bridge)
        .union(wear_pad)
        .union(top_plate)
        .union(left_cheek)
        .union(right_cheek)
        .union(lock_tab)
    )

    left_bore = cq.Workplane("YZ").workplane(offset=-0.002).center(0.000, 0.022).circle(rod_clear).extrude(0.080)
    right_bore = cq.Workplane("YZ").workplane(offset=-0.002).center(0.000, -0.022).circle(rod_clear).extrude(0.080)
    clevis_pin = cq.Workplane("XZ").workplane(offset=-0.030).center(0.076, 0.034).circle(0.0048).extrude(0.060)
    center_relief = cq.Workplane("XY").box(0.040, 0.024, 0.010).translate((0.030, 0.000, 0.018))

    return carriage.cut(left_bore).cut(right_bore).cut(clevis_pin).cut(center_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolute_prismatic_study", assets=ASSETS)

    frame_finish = model.material("frame_finish", rgba=(0.32, 0.34, 0.37, 1.0))
    stage_finish = model.material("stage_finish", rgba=(0.56, 0.58, 0.61, 1.0))
    rod_finish = model.material("rod_finish", rgba=(0.80, 0.82, 0.85, 1.0))
    carriage_finish = model.material("carriage_finish", rgba=(0.42, 0.44, 0.47, 1.0))
    cover_finish = model.material("cover_finish", rgba=(0.47, 0.50, 0.53, 1.0))

    frame = model.part("frame")
    frame.visual(_export_mesh(_frame_shape(), "frame.obj"), material=frame_finish, name="frame_shell")
    frame.inertial = Inertial.from_geometry(
        Box((0.20, 0.11, 0.11)),
        mass=9.5,
        origin=Origin(xyz=(0.040, 0.000, -0.030)),
    )

    swing_stage = model.part("swing_stage")
    swing_stage.visual(
        _export_mesh(_swing_stage_shape(), "swing_stage.obj"),
        material=stage_finish,
        name="swing_stage_shell",
    )
    swing_stage.inertial = Inertial.from_geometry(
        Box((0.23, 0.10, 0.10)),
        mass=5.8,
        origin=Origin(xyz=(0.090, 0.000, 0.020)),
    )

    left_guide_rod = model.part("left_guide_rod")
    left_guide_rod.visual(
        _export_mesh(_guide_rod_shape(), "left_guide_rod.obj"),
        material=rod_finish,
        name="left_guide_rod_shell",
    )
    left_guide_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0075, length=0.164),
        mass=0.38,
        origin=Origin(),
    )

    right_guide_rod = model.part("right_guide_rod")
    right_guide_rod.visual(
        _export_mesh(_guide_rod_shape(), "right_guide_rod.obj"),
        material=rod_finish,
        name="right_guide_rod_shell",
    )
    right_guide_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0075, length=0.164),
        mass=0.38,
        origin=Origin(),
    )

    access_cover = model.part("access_cover")
    access_cover.visual(
        _export_mesh(_access_cover_shape(), "access_cover.obj"),
        material=cover_finish,
        name="access_cover_shell",
    )
    access_cover.inertial = Inertial.from_geometry(
        Box((0.076, 0.0042, 0.038)),
        mass=0.22,
        origin=Origin(),
    )

    carriage = model.part("carriage")
    carriage.visual(
        _export_mesh(_carriage_shape(), "carriage.obj"),
        material=carriage_finish,
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.092, 0.074, 0.064)),
        mass=1.9,
        origin=Origin(xyz=(0.040, 0.000, 0.032)),
    )

    model.articulation(
        "frame_to_swing_stage",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing_stage,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.4, lower=-0.40, upper=0.95),
    )
    model.articulation(
        "swing_stage_to_left_guide_rod",
        ArticulationType.FIXED,
        parent=swing_stage,
        child=left_guide_rod,
        origin=Origin(xyz=(0.112, 0.022, 0.047)),
    )
    model.articulation(
        "swing_stage_to_right_guide_rod",
        ArticulationType.FIXED,
        parent=swing_stage,
        child=right_guide_rod,
        origin=Origin(xyz=(0.112, -0.022, 0.047)),
    )
    model.articulation(
        "swing_stage_to_access_cover",
        ArticulationType.FIXED,
        parent=swing_stage,
        child=access_cover,
        origin=Origin(xyz=(0.096, 0.0295, -0.002)),
    )
    model.articulation(
        "swing_stage_to_carriage",
        ArticulationType.PRISMATIC,
        parent=swing_stage,
        child=carriage,
        origin=Origin(xyz=(0.052, 0.000, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.075),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    swing_stage = object_model.get_part("swing_stage")
    left_guide_rod = object_model.get_part("left_guide_rod")
    right_guide_rod = object_model.get_part("right_guide_rod")
    access_cover = object_model.get_part("access_cover")
    carriage = object_model.get_part("carriage")

    pivot = object_model.get_articulation("frame_to_swing_stage")
    slide = object_model.get_articulation("swing_stage_to_carriage")

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

    ctx.check(
        "pivot_joint_axis_is_y",
        tuple(round(v, 3) for v in pivot.axis) == (0.0, 1.0, 0.0),
        f"Unexpected pivot axis: {pivot.axis}",
    )
    ctx.check(
        "slide_joint_axis_is_x",
        tuple(round(v, 3) for v in slide.axis) == (1.0, 0.0, 0.0),
        f"Unexpected slide axis: {slide.axis}",
    )

    ctx.expect_contact(
        swing_stage,
        frame,
        contact_tol=0.0008,
        name="trunnion_collars_seat_against_frame_cheeks",
    )
    ctx.expect_contact(
        left_guide_rod,
        swing_stage,
        contact_tol=0.0008,
        name="left_guide_rod_is_captured_by_anchor_blocks",
    )
    ctx.expect_contact(
        right_guide_rod,
        swing_stage,
        contact_tol=0.0008,
        name="right_guide_rod_is_captured_by_anchor_blocks",
    )
    ctx.expect_contact(
        access_cover,
        swing_stage,
        contact_tol=0.0008,
        name="access_cover_sits_on_service_land",
    )
    ctx.expect_contact(
        carriage,
        swing_stage,
        contact_tol=0.0008,
        name="carriage_shoe_seats_on_guide_rail",
    )
    ctx.expect_origin_distance(
        left_guide_rod,
        right_guide_rod,
        axes="y",
        min_dist=0.044,
        max_dist=0.044,
        name="guide_rods_keep_disciplined_spacing",
    )
    ctx.expect_overlap(
        carriage,
        swing_stage,
        axes="x",
        min_overlap=0.050,
        name="carriage_remains_substantially_engaged_on_stage_at_rest",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.060}):
        ctx.expect_contact(
            carriage,
            swing_stage,
            contact_tol=0.0008,
            name="carriage_remains_seated_when_extended",
        )
        ctx.expect_overlap(
            carriage,
            swing_stage,
            axes="x",
            min_overlap=0.030,
            name="carriage_keeps_rail_engagement_when_extended",
        )
        extended_carriage = ctx.part_world_position(carriage)

    with ctx.pose({pivot: 0.650}):
        ctx.expect_contact(
            swing_stage,
            frame,
            contact_tol=0.0008,
            name="pivot_hub_stays_captured_through_rotation",
        )
        pitched_carriage = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic_joint_advances_carriage",
        rest_carriage is not None
        and extended_carriage is not None
        and (extended_carriage[0] - rest_carriage[0]) > 0.055
        and abs(extended_carriage[2] - rest_carriage[2]) < 0.002,
        f"Rest carriage={rest_carriage}, extended carriage={extended_carriage}",
    )
    ctx.check(
        "revolute_joint_lifts_stage_and_carriage",
        rest_carriage is not None
        and pitched_carriage is not None
        and pitched_carriage[2] > rest_carriage[2] + 0.045,
        f"Rest carriage={rest_carriage}, pitched carriage={pitched_carriage}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
