from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    fillet: float = 0.0,
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(*size).translate(center)
    if fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid


def _cyl_x(
    radius: float,
    length: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _cyl_y(
    radius: float,
    length: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _make_pedestal_shape() -> cq.Workplane:
    roll_z = 0.150

    base = _rounded_box((0.160, 0.116, 0.016), (0.0, 0.0, 0.008), fillet=0.012)
    plinth = _rounded_box((0.102, 0.086, 0.018), (0.0, 0.0, 0.025), fillet=0.008)
    column = _rounded_box((0.032, 0.022, 0.046), (0.0, 0.0, 0.049), fillet=0.005)
    shoulder = _rounded_box((0.040, 0.028, 0.010), (0.0, 0.0, 0.072), fillet=0.003)

    front_rib = _rounded_box((0.010, 0.010, 0.086), (0.0, 0.032, 0.111), fillet=0.0025)
    rear_rib = _rounded_box((0.010, 0.010, 0.086), (0.0, -0.032, 0.111), fillet=0.0025)

    front_bearing = _rounded_box((0.024, 0.016, 0.028), (0.0, 0.041, roll_z), fillet=0.003)
    rear_bearing = _rounded_box((0.024, 0.016, 0.028), (0.0, -0.041, roll_z), fillet=0.003)
    front_cap = _rounded_box((0.022, 0.004, 0.020), (0.0, 0.051, roll_z), fillet=0.0015)
    rear_cap = _rounded_box((0.022, 0.004, 0.020), (0.0, -0.051, roll_z), fillet=0.0015)
    service_cover = _rounded_box((0.004, 0.022, 0.036), (0.021, 0.0, 0.070), fillet=0.0015)

    pedestal = (
        base.union(plinth)
        .union(column)
        .union(shoulder)
        .union(front_rib)
        .union(rear_rib)
        .union(front_bearing)
        .union(rear_bearing)
        .union(front_cap)
        .union(rear_cap)
        .union(service_cover)
    )

    front_bore = _cyl_y(0.008, 0.024, center=(0.0, 0.041, roll_z))
    rear_bore = _cyl_y(0.008, 0.024, center=(0.0, -0.041, roll_z))
    front_cap_bore = _cyl_y(0.0105, 0.008, center=(0.0, 0.051, roll_z))
    rear_cap_bore = _cyl_y(0.0105, 0.008, center=(0.0, -0.051, roll_z))
    return pedestal.cut(front_bore).cut(rear_bore).cut(front_cap_bore).cut(rear_cap_bore)


def _make_outer_ring_shape() -> cq.Workplane:
    outer_radius = 0.074
    inner_radius = 0.060
    ring_depth = 0.012

    annulus = cq.Workplane("XZ").circle(outer_radius).circle(inner_radius).extrude(ring_depth / 2.0, both=True)
    annulus = (
        annulus.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .circle(0.071)
        .circle(0.064)
        .cutBlind(-0.0015)
    )
    annulus = (
        annulus.faces("<Y")
        .workplane(centerOption="CenterOfMass")
        .circle(0.071)
        .circle(0.064)
        .cutBlind(-0.0015)
    )

    front_journal = _cyl_y(0.008, 0.040, center=(0.0, 0.026, 0.0))
    rear_journal = _cyl_y(0.008, 0.040, center=(0.0, -0.026, 0.0))
    front_retainer = _cyl_y(0.010, 0.008, center=(0.0, 0.042, 0.0))
    rear_retainer = _cyl_y(0.010, 0.008, center=(0.0, -0.042, 0.0))

    left_pod = _rounded_box((0.016, 0.012, 0.020), (-0.054, 0.0, 0.0), fillet=0.002)
    right_pod = _rounded_box((0.016, 0.012, 0.020), (0.054, 0.0, 0.0), fillet=0.002)
    left_cap = _rounded_box((0.004, 0.014, 0.022), (-0.064, 0.0, 0.0), fillet=0.0015)
    right_cap = _rounded_box((0.004, 0.014, 0.022), (0.064, 0.0, 0.0), fillet=0.0015)
    service_cover = _rounded_box((0.016, 0.004, 0.014), (0.044, 0.009, -0.037), fillet=0.0015)

    ring = (
        annulus.union(front_journal)
        .union(rear_journal)
        .union(front_retainer)
        .union(rear_retainer)
        .union(left_pod)
        .union(right_pod)
        .union(left_cap)
        .union(right_cap)
        .union(service_cover)
    )

    left_bore = _cyl_x(0.006, 0.018, center=(-0.054, 0.0, 0.0))
    right_bore = _cyl_x(0.006, 0.018, center=(0.054, 0.0, 0.0))
    left_cap_bore = _cyl_x(0.0085, 0.008, center=(-0.064, 0.0, 0.0))
    right_cap_bore = _cyl_x(0.0085, 0.008, center=(0.064, 0.0, 0.0))
    return ring.cut(left_bore).cut(right_bore).cut(left_cap_bore).cut(right_cap_bore)


def _make_inner_cradle_shape() -> cq.Workplane:
    left_cheek = _rounded_box((0.008, 0.008, 0.040), (-0.020, -0.012, 0.0), fillet=0.0025)
    right_cheek = _rounded_box((0.008, 0.008, 0.040), (0.020, -0.012, 0.0), fillet=0.0025)
    rear_bridge = _rounded_box((0.050, 0.008, 0.012), (0.0, -0.020, -0.014), fillet=0.002)
    upper_strap = _rounded_box((0.044, 0.008, 0.010), (0.0, -0.020, 0.015), fillet=0.002)
    center_spine = _rounded_box((0.016, 0.010, 0.010), (0.0, -0.010, -0.008), fillet=0.0018)
    support_tongue = _rounded_box((0.008, 0.060, 0.006), (0.0, 0.025, 0.006), fillet=0.0012)
    front_pad = _rounded_box((0.018, 0.012, 0.014), (0.0, 0.055, 0.010), fillet=0.0015)
    left_shoulder = _rounded_box((0.010, 0.008, 0.018), (-0.038, -0.006, 0.0), fillet=0.0018)
    right_shoulder = _rounded_box((0.010, 0.008, 0.018), (0.038, -0.006, 0.0), fillet=0.0018)
    left_trunnion = _cyl_x(0.005, 0.024, center=(-0.052, 0.0, 0.0))
    right_trunnion = _cyl_x(0.005, 0.024, center=(0.052, 0.0, 0.0))

    return (
        left_cheek.union(right_cheek)
        .union(rear_bridge)
        .union(upper_strap)
        .union(center_spine)
        .union(support_tongue)
        .union(front_pad)
        .union(left_shoulder)
        .union(right_shoulder)
        .union(left_trunnion)
        .union(right_trunnion)
    )


def _make_tool_plate_shape() -> cq.Workplane:
    mount_block = _rounded_box((0.010, 0.008, 0.008), (0.0, 0.004, 0.0), fillet=0.0012)
    stem = _rounded_box((0.006, 0.040, 0.006), (0.0, 0.040, 0.0), fillet=0.001)
    riser = _rounded_box((0.008, 0.010, 0.026), (0.0, 0.070, 0.013), fillet=0.0012)
    plate_arm = _rounded_box((0.008, 0.018, 0.008), (0.0, 0.079, 0.026), fillet=0.0012)
    gusset = _rounded_box((0.014, 0.012, 0.014), (0.0, 0.070, 0.008), fillet=0.0012)

    plate = cq.Workplane("XY").box(0.028, 0.008, 0.020).translate((0.0, 0.091, 0.026))
    plate = plate.edges("|Y").fillet(0.004)
    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.008, 0.0), (0.008, 0.0)])
        .slot2D(0.008, 0.004, angle=90)
        .cutBlind(-0.008)
    )

    return mount_block.union(stem).union(riser).union(plate_arm).union(gusset).union(plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_instrument_head")

    model.material("pedestal_paint", rgba=(0.70, 0.71, 0.73, 1.0))
    model.material("ring_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    model.material("cradle_gray", rgba=(0.60, 0.62, 0.64, 1.0))
    model.material("tool_dark", rgba=(0.20, 0.22, 0.24, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_make_pedestal_shape(), "pedestal"),
        material="pedestal_paint",
        name="pedestal_shell",
    )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_cadquery(_make_outer_ring_shape(), "outer_ring"),
        material="ring_gray",
        name="outer_ring_shell",
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        mesh_from_cadquery(_make_inner_cradle_shape(), "inner_cradle"),
        material="cradle_gray",
        name="inner_cradle_shell",
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        mesh_from_cadquery(_make_tool_plate_shape(), "tool_plate"),
        material="tool_dark",
        name="tool_plate_shell",
    )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-0.20, upper=0.25),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.08, upper=0.15),
    )
    model.articulation(
        "tool_mount",
        ArticulationType.FIXED,
        parent=inner_cradle,
        child=tool_plate,
        origin=Origin(xyz=(0.0, 0.055, 0.010)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    outer_ring = object_model.get_part("outer_ring")
    inner_cradle = object_model.get_part("inner_cradle")
    tool_plate = object_model.get_part("tool_plate")

    roll_axis = object_model.get_articulation("roll_axis")
    pitch_axis = object_model.get_articulation("pitch_axis")
    tool_mount = object_model.get_articulation("tool_mount")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        pedestal,
        outer_ring,
        reason="roll journals are modeled nested in pedestal bearing housings with zero running clearance",
    )
    ctx.allow_overlap(
        outer_ring,
        inner_cradle,
        reason="pitch trunnions are modeled nested in ring side bearings with zero running clearance",
    )
    ctx.allow_overlap(
        inner_cradle,
        tool_plate,
        reason="the fixed tool plate uses an inset mounting tongue and stem seated into the cradle front pad",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
        "part count matches compact head assembly",
        len(object_model.parts) == 4,
        details=f"expected 4 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "two revolute axes plus fixed tool mount",
        len(object_model.articulations) == 3,
        details=f"expected 3 articulations, found {len(object_model.articulations)}",
    )
    ctx.check(
        "roll axis uses front-to-back shaft",
        roll_axis.axis == (0.0, 1.0, 0.0),
        details=f"roll axis is {roll_axis.axis}",
    )
    ctx.check(
        "pitch axis uses left-to-right shaft",
        pitch_axis.axis == (1.0, 0.0, 0.0),
        details=f"pitch axis is {pitch_axis.axis}",
    )
    ctx.check(
        "tool mount stays fixed",
        tool_mount.articulation_type == ArticulationType.FIXED,
        details=f"tool mount articulation is {tool_mount.articulation_type}",
    )

    ctx.expect_origin_gap(
        outer_ring,
        pedestal,
        axis="z",
        min_gap=0.165,
        max_gap=0.175,
        name="roll ring sits on a compact pedestal height",
    )

    ctx.expect_contact(
        pedestal,
        outer_ring,
        contact_tol=0.001,
        name="roll package is physically seated in the pedestal housing",
    )
    ctx.expect_contact(
        outer_ring,
        inner_cradle,
        contact_tol=0.001,
        name="pitch package is physically seated in the outer ring",
    )
    ctx.expect_contact(
        inner_cradle,
        tool_plate,
        contact_tol=0.001,
        name="tool plate is mounted to the cradle pad",
    )

    with ctx.pose({roll_axis: roll_axis.motion_limits.upper, pitch_axis: pitch_axis.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no clipping at positive roll and pitch travel")

    with ctx.pose({roll_axis: roll_axis.motion_limits.lower, pitch_axis: pitch_axis.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no clipping at negative roll and pitch travel")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
