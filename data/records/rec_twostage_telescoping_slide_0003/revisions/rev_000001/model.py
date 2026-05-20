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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

OUTER_L = 0.400
OUTER_W = 0.078
OUTER_H = 0.060
OUTER_WALL = 0.0045

MID_L = 0.308
MID_W = 0.060
MID_H = 0.044
MID_WALL = 0.0040

INNER_L = 0.238
INNER_W = 0.042
INNER_H = 0.028
INNER_WALL = 0.0035

OUTER_Z = 0.074


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((cx - (length / 2.0), cy, cz))
    )


def _tube_with_windows(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    side_slot_length: float,
    side_slot_height: float,
    top_window_length: float,
    top_window_width: float,
    latch_hole_x: tuple[float, ...],
) -> cq.Workplane:
    tube = cq.Workplane("XY").box(length, width, height)
    tube = tube.cut(cq.Workplane("XY").box(length + 0.012, width - (2.0 * wall), height - (2.0 * wall)))

    side_cutter = cq.Workplane("XY").box(side_slot_length, wall * 6.0, side_slot_height)
    side_offset = (width / 2.0) - (wall * 1.25)
    tube = tube.cut(side_cutter.translate((0.0, side_offset, 0.0)))
    tube = tube.cut(side_cutter.translate((0.0, -side_offset, 0.0)))

    top_cutter = cq.Workplane("XY").box(top_window_length, top_window_width, wall * 5.0)
    tube = tube.cut(top_cutter.translate((0.0, 0.0, (height / 2.0) - (wall * 1.2))))

    if latch_hole_x:
        tube = (
            tube.faces(">Z")
            .workplane(centerOption="CenterOfMass")
            .pushPoints([(x_pos, 0.0) for x_pos in latch_hole_x])
            .hole(0.006)
        )
    return tube


def _support_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(0.320, 0.122, 0.012).translate((0.0, 0.0, 0.006))
    frame = frame.union(cq.Workplane("XY").box(0.245, 0.046, 0.016).translate((0.0, 0.0, 0.020)))
    frame = frame.union(cq.Workplane("XY").box(0.060, 0.094, 0.026).translate((-0.115, 0.0, 0.025)))
    frame = frame.union(cq.Workplane("XY").box(0.060, 0.094, 0.026).translate((0.115, 0.0, 0.025)))
    frame = frame.union(cq.Workplane("XY").box(0.240, 0.020, 0.020).translate((0.0, 0.0, 0.032)))

    for x_pos in (-0.115, 0.115):
        for y_pos in (-0.036, 0.028):
            gusset = (
                cq.Workplane("XZ")
                .polyline([(-0.024, 0.012), (0.024, 0.012), (0.024, 0.038)])
                .close()
                .extrude(0.008)
                .translate((x_pos, y_pos, 0.0))
            )
            frame = frame.union(gusset)

    frame = (
        frame.faces("<Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.135, -0.040), (-0.135, 0.040), (0.135, -0.040), (0.135, 0.040)])
        .hole(0.010)
    )
    return frame


def _mount_bracket_shape() -> cq.Workplane:
    bracket = cq.Workplane("XY").box(0.040, 0.130, 0.006).translate((0.0, 0.0, -0.077))
    bracket = bracket.union(
        cq.Workplane("XY").box(0.040, 0.006, 0.092).translate((0.0, 0.052, -0.034))
    )
    bracket = bracket.union(
        cq.Workplane("XY").box(0.040, 0.006, 0.092).translate((0.0, -0.052, -0.034))
    )
    bracket = bracket.union(
        cq.Workplane("XY").box(0.040, 0.018, 0.004).translate((0.0, 0.046, -0.032))
    )
    bracket = bracket.union(
        cq.Workplane("XY").box(0.040, 0.018, 0.004).translate((0.0, -0.046, -0.032))
    )

    for y_pos in (-0.052, 0.052):
        bracket = bracket.cut(_x_cylinder(0.0032, 0.050, (0.0, y_pos, -0.058)))
    return bracket


def _cover_plate_shape(length: float, width: float, thickness: float) -> cq.Workplane:
    cover = cq.Workplane("XY").box(length, width, thickness)
    boss_offsets = (
        (-length * 0.34, -width * 0.30),
        (-length * 0.34, width * 0.30),
        (length * 0.34, -width * 0.30),
        (length * 0.34, width * 0.30),
    )
    for x_pos, y_pos in boss_offsets:
        cover = cover.union(
            cq.Workplane("XY").circle(0.003).extrude(0.0025).translate((x_pos, y_pos, thickness / 2.0))
        )
    return cover


def _rear_end_cap_shape(width: float, height: float, inner_width: float, inner_height: float) -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.004, width, height)
    cap = cap.union(cq.Workplane("XY").box(0.010, width - 0.012, 0.006).translate((-0.007, 0.0, (height / 2.0) - 0.003)))
    cap = cap.union(cq.Workplane("XY").box(0.010, width - 0.012, 0.006).translate((-0.007, 0.0, -((height / 2.0) - 0.003))))
    return cap


def _guide_frame_shape(
    *,
    carrier_length: float,
    carrier_width: float,
    carrier_height: float,
    bridge_width: float,
    roller_y: float,
    roller_radius: float,
    roller_x: tuple[float, float],
    roller_z: tuple[float, float],
) -> cq.Workplane:
    base_z = (carrier_height / 2.0) + 0.0015
    frame = cq.Workplane("XY").box(carrier_length, bridge_width, 0.003).translate((0.0, 0.0, base_z))
    side_height = 0.020
    side_center_z = base_z + (side_height / 2.0) + 0.0015
    side_y = (bridge_width / 2.0) - 0.0025
    frame = frame.union(cq.Workplane("XY").box(carrier_length, 0.005, side_height).translate((0.0, side_y, side_center_z)))
    frame = frame.union(cq.Workplane("XY").box(carrier_length, 0.005, side_height).translate((0.0, -side_y, side_center_z)))

    for x_pos in roller_x:
        for z_pos in roller_z:
            roller_center_z = base_z + 0.010 + z_pos
            frame = frame.union(_x_cylinder(roller_radius, 0.010, (x_pos, roller_y, roller_center_z)))
            frame = frame.union(_x_cylinder(roller_radius, 0.010, (x_pos, -roller_y, roller_center_z)))
    return frame


def _output_bracket_shape() -> cq.Workplane:
    bracket = cq.Workplane("XY").box(0.038, 0.024, 0.003).translate((0.0, 0.0, (INNER_H / 2.0) + 0.0015))
    bracket = bracket.union(cq.Workplane("XY").box(0.006, 0.004, 0.030).translate((0.0, 0.010, (INNER_H / 2.0) + 0.018)))
    bracket = bracket.union(cq.Workplane("XY").box(0.006, 0.004, 0.030).translate((0.0, -0.010, (INNER_H / 2.0) + 0.018)))
    bracket = bracket.union(cq.Workplane("XY").box(0.010, 0.024, 0.008).translate((0.014, 0.0, (INNER_H / 2.0) + 0.008)))
    bracket = bracket.cut(cq.Workplane("XZ").circle(0.0045).extrude(0.030).translate((0.0, -0.015, (INNER_H / 2.0) + 0.018)))
    return bracket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.23, 0.25, 0.27, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.61, 0.64, 0.67, 1.0))
    zinc = model.material("zinc", rgba=(0.74, 0.76, 0.79, 1.0))
    oxide = model.material("oxide", rgba=(0.16, 0.17, 0.18, 1.0))
    polymer = model.material("polymer", rgba=(0.10, 0.10, 0.10, 1.0))
    wear = model.material("wear_strip", rgba=(0.72, 0.62, 0.36, 1.0))

    support = model.part("support_frame")
    support.visual(
        mesh_from_cadquery(_support_frame_shape(), "support_frame.obj", assets=ASSETS),
        name="support_structure",
        material=painted_steel,
    )
    support.inertial = Inertial.from_geometry(Box((0.320, 0.122, 0.050)), mass=3.8, origin=Origin(xyz=(0.0, 0.0, 0.020)))

    outer = model.part("outer_member")
    outer.visual(
        mesh_from_cadquery(
            _tube_with_windows(
                length=OUTER_L,
                width=OUTER_W,
                height=OUTER_H,
                wall=OUTER_WALL,
                side_slot_length=0.240,
                side_slot_height=0.030,
                top_window_length=0.170,
                top_window_width=0.044,
                latch_hole_x=(-0.120, -0.040, 0.040, 0.120),
            ),
            "outer_member.obj",
            assets=ASSETS,
        ),
        name="outer_shell",
        material=oxide,
    )
    outer.inertial = Inertial.from_geometry(Box((OUTER_L, OUTER_W, OUTER_H)), mass=2.2)
    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=support,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, OUTER_Z)),
    )

    front_bracket = model.part("front_mount_bracket")
    front_bracket.visual(
        mesh_from_cadquery(_mount_bracket_shape(), "front_mount_bracket.obj", assets=ASSETS),
        name="front_bracket_body",
        material=machined_steel,
    )
    front_bracket.inertial = Inertial.from_geometry(Box((0.040, 0.098, 0.040)), mass=0.24, origin=Origin(xyz=(0.0, 0.0, -0.013)))
    model.articulation(
        "outer_to_front_bracket",
        ArticulationType.FIXED,
        parent=outer,
        child=front_bracket,
        origin=Origin(xyz=(-0.115, 0.0, 0.0)),
    )

    rear_bracket = model.part("rear_mount_bracket")
    rear_bracket.visual(
        mesh_from_cadquery(_mount_bracket_shape(), "rear_mount_bracket.obj", assets=ASSETS),
        name="rear_bracket_body",
        material=machined_steel,
    )
    rear_bracket.inertial = Inertial.from_geometry(Box((0.040, 0.098, 0.040)), mass=0.24, origin=Origin(xyz=(0.0, 0.0, -0.013)))
    model.articulation(
        "outer_to_rear_bracket",
        ArticulationType.FIXED,
        parent=outer,
        child=rear_bracket,
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
    )

    outer_cover = model.part("outer_access_cover")
    outer_cover.visual(
        mesh_from_cadquery(_cover_plate_shape(0.170, 0.050, 0.003), "outer_access_cover.obj", assets=ASSETS),
        name="outer_cover_plate",
        material=zinc,
    )
    outer_cover.inertial = Inertial.from_geometry(Box((0.170, 0.050, 0.006)), mass=0.12)
    model.articulation(
        "outer_to_access_cover",
        ArticulationType.FIXED,
        parent=outer,
        child=outer_cover,
        origin=Origin(xyz=(0.0, 0.0, (OUTER_H / 2.0) + 0.0015)),
    )

    outer_rear_cap = model.part("outer_rear_cap")
    outer_rear_cap.visual(
        mesh_from_cadquery(
            _rear_end_cap_shape(
                OUTER_W,
                OUTER_H,
                OUTER_W - (2.0 * OUTER_WALL),
                OUTER_H - (2.0 * OUTER_WALL),
            ),
            "outer_rear_cap.obj",
            assets=ASSETS,
        ),
        name="outer_rear_cap_body",
        material=polymer,
    )
    outer_rear_cap.inertial = Inertial.from_geometry(Box((0.016, OUTER_W, OUTER_H)), mass=0.06, origin=Origin(xyz=(0.008, 0.0, 0.0)))
    model.articulation(
        "outer_to_rear_cap",
        ArticulationType.FIXED,
        parent=outer,
        child=outer_rear_cap,
        origin=Origin(xyz=(-OUTER_L / 2.0, 0.0, 0.0)),
    )

    stage1 = model.part("intermediate_member")
    stage1.visual(
        mesh_from_cadquery(
            _tube_with_windows(
                length=MID_L,
                width=MID_W,
                height=MID_H,
                wall=MID_WALL,
                side_slot_length=0.170,
                side_slot_height=0.022,
                top_window_length=0.102,
                top_window_width=0.030,
                latch_hole_x=(-0.080, 0.0, 0.080),
            ),
            "intermediate_member.obj",
            assets=ASSETS,
        ),
        name="stage_shell",
        material=machined_steel,
    )
    stage1.visual(
        Box((0.200, MID_W - 0.012, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, (MID_H / 2.0) + 0.0006)),
        name="upper_wear_strip",
        material=wear,
    )
    stage1.visual(
        Box((0.200, MID_W - 0.012, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, -((MID_H / 2.0) + 0.0006))),
        name="lower_wear_strip",
        material=wear,
    )
    stage1.inertial = Inertial.from_geometry(Box((MID_L, MID_W, MID_H)), mass=1.3)
    model.articulation(
        "outer_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.4, lower=0.0, upper=0.150),
    )

    stage1_guides = model.part("intermediate_guides")
    stage1_guides.visual(
        mesh_from_cadquery(
            _guide_frame_shape(
                carrier_length=0.235,
                carrier_width=MID_W,
                carrier_height=MID_H,
                bridge_width=0.062,
                roller_y=0.0322,
                roller_radius=0.0022,
                roller_x=(-0.092, 0.092),
                roller_z=(-0.012, 0.012),
            ),
            "intermediate_guides.obj",
            assets=ASSETS,
        ),
        name="guide_frame",
        material=oxide,
    )
    stage1_guides.inertial = Inertial.from_geometry(Box((0.235, 0.066, 0.030)), mass=0.18, origin=Origin(xyz=(0.0, 0.0, 0.012)))
    model.articulation(
        "intermediate_to_guides",
        ArticulationType.FIXED,
        parent=stage1,
        child=stage1_guides,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    stage_cover = model.part("intermediate_access_cover")
    stage_cover.visual(
        mesh_from_cadquery(_cover_plate_shape(0.100, 0.034, 0.003), "intermediate_access_cover.obj", assets=ASSETS),
        name="stage_cover_plate",
        material=zinc,
    )
    stage_cover.inertial = Inertial.from_geometry(Box((0.100, 0.034, 0.006)), mass=0.07)
    model.articulation(
        "intermediate_to_cover",
        ArticulationType.FIXED,
        parent=stage1,
        child=stage_cover,
        origin=Origin(xyz=(0.0, 0.0, (MID_H / 2.0) + 0.0015)),
    )

    output = model.part("output_member")
    output.visual(
        mesh_from_cadquery(
            _tube_with_windows(
                length=INNER_L,
                width=INNER_W,
                height=INNER_H,
                wall=INNER_WALL,
                side_slot_length=0.124,
                side_slot_height=0.016,
                top_window_length=0.082,
                top_window_width=0.020,
                latch_hole_x=(-0.050, 0.0, 0.050),
            ),
            "output_member.obj",
            assets=ASSETS,
        ),
        name="output_shell",
        material=zinc,
    )
    output.visual(
        Box((0.142, INNER_W - 0.008, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, (INNER_H / 2.0) + 0.00075)),
        name="output_upper_wear_strip",
        material=wear,
    )
    output.visual(
        Box((0.142, INNER_W - 0.008, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, -((INNER_H / 2.0) + 0.00075))),
        name="output_lower_wear_strip",
        material=wear,
    )
    output.inertial = Inertial.from_geometry(Box((INNER_L, INNER_W, INNER_H)), mass=0.82)
    model.articulation(
        "intermediate_to_output",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=output,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.4, lower=0.0, upper=0.125),
    )

    output_guides = model.part("output_guides")
    output_guides.visual(
        mesh_from_cadquery(
            _guide_frame_shape(
                carrier_length=0.170,
                carrier_width=INNER_W,
                carrier_height=INNER_H,
                bridge_width=0.046,
                roller_y=0.0223,
                roller_radius=0.0035,
                roller_x=(-0.058, 0.058),
                roller_z=(-0.008, 0.008),
            ),
            "output_guides.obj",
            assets=ASSETS,
        ),
        name="output_guide_frame",
        material=oxide,
    )
    output_guides.inertial = Inertial.from_geometry(Box((0.170, 0.048, 0.022)), mass=0.14, origin=Origin(xyz=(0.0, 0.0, 0.008)))
    model.articulation(
        "output_to_guides",
        ArticulationType.FIXED,
        parent=output,
        child=output_guides,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    output_bracket = model.part("output_bracket")
    output_bracket.visual(
        mesh_from_cadquery(_output_bracket_shape(), "output_bracket.obj", assets=ASSETS),
        name="clevis_bracket",
        material=machined_steel,
    )
    output_bracket.inertial = Inertial.from_geometry(Box((0.052, 0.056, 0.044)), mass=0.22)
    model.articulation(
        "output_to_bracket",
        ArticulationType.FIXED,
        parent=output,
        child=output_bracket,
        origin=Origin(xyz=((INNER_L / 2.0) + 0.026, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support = object_model.get_part("support_frame")
    outer = object_model.get_part("outer_member")
    front_bracket = object_model.get_part("front_mount_bracket")
    rear_bracket = object_model.get_part("rear_mount_bracket")
    outer_cover = object_model.get_part("outer_access_cover")
    outer_rear_cap = object_model.get_part("outer_rear_cap")
    stage1 = object_model.get_part("intermediate_member")
    stage1_guides = object_model.get_part("intermediate_guides")
    stage_cover = object_model.get_part("intermediate_access_cover")
    output = object_model.get_part("output_member")
    output_guides = object_model.get_part("output_guides")
    output_bracket = object_model.get_part("output_bracket")

    outer_to_stage1 = object_model.get_articulation("outer_to_intermediate")
    stage1_to_output = object_model.get_articulation("intermediate_to_output")

    outer_shell = outer.get_visual("outer_shell")
    stage_shell = stage1.get_visual("stage_shell")
    output_shell = output.get_visual("output_shell")

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

    ctx.expect_contact(front_bracket, outer, name="front bracket clamps outer member")
    ctx.expect_contact(front_bracket, support, name="front bracket seats on support")
    ctx.expect_contact(rear_bracket, outer, name="rear bracket clamps outer member")
    ctx.expect_contact(rear_bracket, support, name="rear bracket seats on support")
    ctx.expect_contact(outer_cover, outer, elem_b=outer_shell, name="outer cover sits on outer shell")
    ctx.expect_contact(outer_rear_cap, outer, elem_b=outer_shell, name="outer rear cap closes outer member")
    ctx.expect_contact(stage1_guides, stage1, elem_b=stage_shell, name="intermediate guide frame mounts to stage")
    ctx.expect_contact(stage_cover, stage1, elem_b=stage_shell, name="intermediate access cover mounts to stage")
    ctx.expect_contact(output_guides, output, elem_b=output_shell, name="output guide frame mounts to output")
    ctx.expect_contact(output_bracket, output, elem_b=output_shell, name="output bracket mounts to output member")

    ctx.expect_origin_gap(stage1, outer, axis="x", min_gap=0.0, max_gap=0.001, name="stage one starts retracted")
    ctx.expect_origin_gap(output, stage1, axis="x", min_gap=0.0, max_gap=0.001, name="output stage starts retracted")
    ctx.expect_overlap(stage1, outer, axes="x", min_overlap=0.300, name="stage one retains deep overlap at rest")
    ctx.expect_overlap(output, stage1, axes="x", min_overlap=0.230, name="output stage retains deep overlap at rest")
    ctx.expect_within(stage1, outer, axes="yz", margin=0.0, name="stage one stays nested inside outer envelope")
    ctx.expect_within(output, stage1, axes="yz", margin=0.0, name="output stage stays nested inside intermediate envelope")
    ctx.expect_within(stage1_guides, outer, axes="yz", margin=0.0, name="intermediate guide frame stays within outer sleeve")
    ctx.expect_within(output_guides, stage1, axes="yz", margin=0.0, name="output guide frame stays within intermediate sleeve")

    with ctx.pose({outer_to_stage1: 0.150, stage1_to_output: 0.125}):
        ctx.expect_origin_gap(stage1, outer, axis="x", min_gap=0.149, max_gap=0.151, name="stage one extends along slide axis")
        ctx.expect_origin_gap(output, stage1, axis="x", min_gap=0.124, max_gap=0.126, name="output stage extends from intermediate")
        ctx.expect_overlap(stage1, outer, axes="x", min_overlap=0.154, name="stage one keeps believable overlap when extended")
        ctx.expect_overlap(output, stage1, axes="x", min_overlap=0.145, name="output stage keeps believable overlap when extended")
        ctx.expect_within(stage1_guides, outer, axes="yz", margin=0.0, name="intermediate guide frame remains captured when extended")
        ctx.expect_within(output_guides, stage1, axes="yz", margin=0.0, name="output guide frame remains captured when extended")
        ctx.expect_origin_gap(output_bracket, outer, axis="x", min_gap=0.400, name="output bracket projects beyond stationary outer member")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
