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

WALL_PLATE_T = 0.008
WALL_PLATE_W = 0.090
WALL_PLATE_H = 0.120
WALL_STANDOFF_L = 0.012
WALL_STANDOFF_W = 0.042
WALL_STANDOFF_H = 0.032

KNUCKLE_R = 0.016
KNUCKLE_H = 0.018

ARM_W = 0.038
ARM_H = 0.018
ARM_WINDOW_W = 0.018
ARM1_LEN = 0.090
ARM2_LEN = 0.082

HEAD_BODY_L = 0.022
HEAD_BODY_W = 0.040
HEAD_BODY_H = 0.024
HEAD_HINGE_X = 0.042
CLEVIS_L = 0.018
CLEVIS_W = 0.055
CLEVIS_H = 0.040
EAR_W = 0.012
FRAME_BRIDGE_T = 0.008
HINGE_FLANGE_T = 0.002
HINGE_FLANGE_R = 0.007
HINGE_LUG_T = 0.006
HINGE_LUG_H = 0.028

FACEPLATE_T = 0.006
FACEPLATE_W = 0.125
FACEPLATE_H = 0.095
FACEPLATE_X = 0.022
BARREL_R = 0.009
BARREL_LEN = 0.021


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").cylinder(length, radius).translate((cx, cy, cz))


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((cx, cy, cz))
    )


def _arm_beam(start_x: float, end_x: float, width: float, height: float) -> cq.Workplane:
    length = end_x - start_x
    outer = _box((length, width, height), ((start_x + end_x) / 2.0, 0.0, 0.0))
    inner_length = max(length - 0.018, 0.010)
    inner = _box((inner_length, ARM_WINDOW_W, height + 0.004), ((start_x + end_x) / 2.0, 0.0, 0.0))
    return outer.cut(inner)


def _add_visual(part, shape: cq.Workplane, filename: str, name: str, material) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material, name=name)


def _make_wall_plate() -> cq.Workplane:
    plate = _box(
        (WALL_PLATE_T, WALL_PLATE_W, WALL_PLATE_H),
        (WALL_PLATE_T / 2.0, 0.0, 0.0),
    )
    slot = _box((WALL_PLATE_T * 1.6, 0.010, 0.028), (WALL_PLATE_T / 2.0, 0.0, 0.0))
    plate = plate.cut(slot.translate((0.0, 0.0, 0.034)))
    plate = plate.cut(slot.translate((0.0, 0.0, -0.034)))
    return plate


def _make_wall_standoff() -> cq.Workplane:
    return _box(
        (WALL_STANDOFF_L, WALL_STANDOFF_W, WALL_STANDOFF_H),
        (0.014, 0.0, -0.002),
    )


def _make_wall_knuckle() -> cq.Workplane:
    return _cyl_z(KNUCKLE_R, KNUCKLE_H, (0.036, 0.0, -(KNUCKLE_H / 2.0)))


def _make_arm(rear_upper: bool, front_upper: bool, joint_distance: float) -> dict[str, cq.Workplane]:
    rear_z = KNUCKLE_H / 2.0 if rear_upper else -(KNUCKLE_H / 2.0)
    front_z = KNUCKLE_H / 2.0 if front_upper else -(KNUCKLE_H / 2.0)
    beam_start = KNUCKLE_R
    beam_end = joint_distance - KNUCKLE_R
    return {
        "rear_knuckle": _cyl_z(KNUCKLE_R, KNUCKLE_H, (0.0, 0.0, rear_z)),
        "arm_beam": _arm_beam(beam_start, beam_end, ARM_W, ARM_H),
        "front_knuckle": _cyl_z(KNUCKLE_R, KNUCKLE_H, (joint_distance, 0.0, front_z)),
    }


def _make_head_body() -> dict[str, cq.Workplane]:
    rear_knuckle = _cyl_z(KNUCKLE_R, KNUCKLE_H, (0.0, 0.0, KNUCKLE_H / 2.0))
    rear_riser = _box((0.012, 0.018, 0.020), (0.010, 0.0, 0.010))
    top_strap = _box((0.022, 0.018, 0.010), (0.019, 0.0, 0.016))
    left_ear = _box((CLEVIS_L, EAR_W, CLEVIS_H), (HEAD_HINGE_X - 0.002, -0.0175, 0.0))
    right_ear = _box((CLEVIS_L, EAR_W, CLEVIS_H), (HEAD_HINGE_X - 0.002, 0.0175, 0.0))
    top_bridge = _box((0.010, CLEVIS_W, FRAME_BRIDGE_T), (HEAD_HINGE_X - 0.008, 0.0, 0.016))
    bottom_bridge = _box((0.010, CLEVIS_W, FRAME_BRIDGE_T), (HEAD_HINGE_X - 0.008, 0.0, -0.016))
    clevis = left_ear.union(right_ear).union(top_bridge).union(bottom_bridge)
    head_body = rear_riser.union(top_strap)
    return {
        "rear_knuckle": rear_knuckle,
        "head_body": head_body,
        "clevis": clevis,
    }


def _make_faceplate() -> dict[str, cq.Workplane]:
    barrel = _cyl_y(0.0045, BARREL_LEN, (0.014, 0.0, 0.0))
    left_lug = _box((0.006, HINGE_LUG_T, HINGE_LUG_H), (0.010, -0.0265, 0.0))
    right_lug = _box((0.006, HINGE_LUG_T, HINGE_LUG_H), (0.010, 0.0265, 0.0))
    cross_bridge = _box((0.016, 0.059, 0.010), (0.018, 0.0, 0.0))
    bridge = left_lug.union(right_lug).union(cross_bridge)
    panel = _box((FACEPLATE_T, FACEPLATE_W, FACEPLATE_H), (FACEPLATE_X, 0.0, 0.0))
    for y_sign in (-1.0, 1.0):
        for z_sign in (-1.0, 1.0):
            panel = panel.cut(
                _box(
                    (FACEPLATE_T * 2.0, 0.010, 0.018),
                    (FACEPLATE_X, y_sign * 0.045, z_sign * 0.028),
                )
            )
    return {
        "hinge_barrel": barrel,
        "hinge_bridge": bridge,
        "faceplate_panel": panel,
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_tv_wall_mount", assets=ASSETS)

    black_oxide = model.material("black_oxide", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.48, 0.49, 0.50, 1.0))

    wall_bracket = model.part("wall_bracket")
    _add_visual(wall_bracket, _make_wall_plate(), "wall_plate.obj", "wall_plate", graphite)
    _add_visual(wall_bracket, _make_wall_standoff(), "wall_standoff.obj", "standoff", graphite)
    _add_visual(wall_bracket, _make_wall_knuckle(), "wall_knuckle.obj", "pivot_block", black_oxide)
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.040, WALL_PLATE_W, WALL_PLATE_H)),
        mass=1.2,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    primary_arm = model.part("primary_arm")
    primary_shapes = _make_arm(rear_upper=True, front_upper=False, joint_distance=ARM1_LEN)
    _add_visual(primary_arm, primary_shapes["rear_knuckle"], "primary_rear_knuckle.obj", "rear_knuckle", black_oxide)
    _add_visual(primary_arm, primary_shapes["arm_beam"], "primary_beam.obj", "arm_beam", satin_steel)
    _add_visual(primary_arm, primary_shapes["front_knuckle"], "primary_front_knuckle.obj", "front_knuckle", black_oxide)
    primary_arm.inertial = Inertial.from_geometry(
        Box((ARM1_LEN, ARM_W, 0.036)),
        mass=1.0,
        origin=Origin(xyz=(ARM1_LEN / 2.0, 0.0, 0.0)),
    )

    secondary_arm = model.part("secondary_arm")
    secondary_shapes = _make_arm(rear_upper=True, front_upper=False, joint_distance=ARM2_LEN)
    _add_visual(secondary_arm, secondary_shapes["rear_knuckle"], "secondary_rear_knuckle.obj", "rear_knuckle", black_oxide)
    _add_visual(secondary_arm, secondary_shapes["arm_beam"], "secondary_beam.obj", "arm_beam", satin_steel)
    _add_visual(secondary_arm, secondary_shapes["front_knuckle"], "secondary_front_knuckle.obj", "front_knuckle", black_oxide)
    secondary_arm.inertial = Inertial.from_geometry(
        Box((ARM2_LEN, ARM_W, 0.036)),
        mass=0.9,
        origin=Origin(xyz=(ARM2_LEN / 2.0, 0.0, 0.0)),
    )

    swivel_head = model.part("swivel_head")
    head_shapes = _make_head_body()
    _add_visual(swivel_head, head_shapes["rear_knuckle"], "head_rear_knuckle.obj", "rear_knuckle", black_oxide)
    _add_visual(swivel_head, head_shapes["head_body"], "head_body.obj", "head_body", graphite)
    _add_visual(swivel_head, head_shapes["clevis"], "head_clevis.obj", "clevis", black_oxide)
    swivel_head.inertial = Inertial.from_geometry(
        Box((0.060, CLEVIS_W, CLEVIS_H)),
        mass=0.7,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    faceplate = model.part("faceplate")
    faceplate_shapes = _make_faceplate()
    _add_visual(faceplate, faceplate_shapes["hinge_barrel"], "faceplate_barrel.obj", "hinge_barrel", black_oxide)
    _add_visual(faceplate, faceplate_shapes["hinge_bridge"], "faceplate_bridge.obj", "hinge_bridge", graphite)
    _add_visual(faceplate, faceplate_shapes["faceplate_panel"], "faceplate_panel.obj", "faceplate_panel", graphite)
    faceplate.inertial = Inertial.from_geometry(
        Box((0.026, FACEPLATE_W, FACEPLATE_H)),
        mass=0.8,
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
    )

    model.articulation(
        "wall_to_primary",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=primary_arm,
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.9, upper=1.9),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=secondary_arm,
        origin=Origin(xyz=(ARM1_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.2, lower=-2.3, upper=2.3),
    )
    model.articulation(
        "secondary_to_swivel",
        ArticulationType.REVOLUTE,
        parent=secondary_arm,
        child=swivel_head,
        origin=Origin(xyz=(ARM2_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.6, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "swivel_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=swivel_head,
        child=faceplate,
        origin=Origin(xyz=(HEAD_HINGE_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.45, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    wall_bracket = object_model.get_part("wall_bracket")
    primary_arm = object_model.get_part("primary_arm")
    secondary_arm = object_model.get_part("secondary_arm")
    swivel_head = object_model.get_part("swivel_head")
    faceplate = object_model.get_part("faceplate")

    wall_to_primary = object_model.get_articulation("wall_to_primary")
    primary_to_secondary = object_model.get_articulation("primary_to_secondary")
    secondary_to_swivel = object_model.get_articulation("secondary_to_swivel")
    swivel_to_faceplate = object_model.get_articulation("swivel_to_faceplate")

    wall_pivot = wall_bracket.get_visual("pivot_block")
    primary_rear = primary_arm.get_visual("rear_knuckle")
    primary_front = primary_arm.get_visual("front_knuckle")
    secondary_rear = secondary_arm.get_visual("rear_knuckle")
    secondary_front = secondary_arm.get_visual("front_knuckle")
    head_rear = swivel_head.get_visual("rear_knuckle")
    clevis = swivel_head.get_visual("clevis")
    barrel = faceplate.get_visual("hinge_barrel")
    bridge = faceplate.get_visual("hinge_bridge")
    panel = faceplate.get_visual("faceplate_panel")

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
    ctx.expect_contact(primary_arm, wall_bracket, elem_a=primary_rear, elem_b=wall_pivot, name="wall joint has stacked knuckles in contact")
    ctx.expect_overlap(primary_arm, wall_bracket, axes="xy", elem_a=primary_rear, elem_b=wall_pivot, min_overlap=0.028, name="wall joint knuckles share pivot axis")
    ctx.expect_gap(
        primary_arm,
        wall_bracket,
        axis="z",
        positive_elem=primary_rear,
        negative_elem=wall_pivot,
        max_gap=0.0005,
        max_penetration=0.0,
        name="wall joint stack seats without penetration",
    )

    ctx.expect_contact(primary_arm, secondary_arm, elem_a=primary_front, elem_b=secondary_rear, name="elbow joint has stacked knuckles in contact")
    ctx.expect_overlap(primary_arm, secondary_arm, axes="xy", elem_a=primary_front, elem_b=secondary_rear, min_overlap=0.028, name="elbow joint knuckles share pivot axis")
    ctx.expect_gap(
        secondary_arm,
        primary_arm,
        axis="z",
        positive_elem=secondary_rear,
        negative_elem=primary_front,
        max_gap=0.0005,
        max_penetration=0.0,
        name="elbow joint stack seats without penetration",
    )

    ctx.expect_contact(secondary_arm, swivel_head, elem_a=secondary_front, elem_b=head_rear, name="head swivel joint has stacked knuckles in contact")
    ctx.expect_overlap(secondary_arm, swivel_head, axes="xy", elem_a=secondary_front, elem_b=head_rear, min_overlap=0.028, name="head swivel knuckles share pivot axis")
    ctx.expect_gap(
        swivel_head,
        secondary_arm,
        axis="z",
        positive_elem=head_rear,
        negative_elem=secondary_front,
        max_gap=0.0005,
        max_penetration=0.0,
        name="head swivel stack seats without penetration",
    )

    ctx.expect_contact(faceplate, swivel_head, elem_a=bridge, elem_b=clevis, contact_tol=0.0005, name="tilt side plates bear on clevis ears")
    ctx.expect_within(faceplate, swivel_head, axes="yz", inner_elem=barrel, outer_elem=clevis, margin=0.010, name="tilt barrel remains captured inside support frame silhouette")
    ctx.expect_origin_distance(faceplate, swivel_head, axes="yz", max_dist=0.001, name="faceplate hinge stays centered on head")
    ctx.expect_overlap(faceplate, swivel_head, axes="yz", elem_a=barrel, elem_b=clevis, min_overlap=0.0085, name="tilt hinge pin remains laterally engaged inside clevis")
    ctx.expect_within(faceplate, faceplate, axes="yz", inner_elem=barrel, outer_elem=panel, margin=0.070, name="faceplate panel is substantially larger than hinge barrel")

    with ctx.pose(
        {
            wall_to_primary: 0.85,
            primary_to_secondary: -1.10,
            secondary_to_swivel: 0.22,
            swivel_to_faceplate: -0.22,
        }
    ):
        ctx.expect_contact(primary_arm, wall_bracket, elem_a=primary_rear, elem_b=wall_pivot, name="wall joint remains seated in open pose")
        ctx.expect_contact(primary_arm, secondary_arm, elem_a=primary_front, elem_b=secondary_rear, name="elbow joint remains seated in open pose")
        ctx.expect_contact(secondary_arm, swivel_head, elem_a=secondary_front, elem_b=head_rear, name="head swivel remains seated in open pose")
        ctx.expect_contact(faceplate, swivel_head, elem_a=bridge, elem_b=clevis, contact_tol=0.0005, name="tilt side plates remain seated on clevis ears in open pose")
        ctx.expect_origin_distance(faceplate, swivel_head, axes="yz", max_dist=0.0015, name="tilted faceplate still pivots about horizontal hinge axis")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
