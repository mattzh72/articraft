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


BASE_PLATE_X = 0.58
BASE_PLATE_Y = 0.46
BASE_PLATE_Z = 0.028
SKID_X = 0.09
SKID_Y = 0.34
SKID_Z = 0.035

MAST_X = 0.16
MAST_Y = 0.14
MAST_Z = 1.34
MAST_BASE_Z = SKID_Z + BASE_PLATE_Z

UPPER_AXIS = (0.0, 0.126, 1.27)
LOWER_AXIS = (0.136, 0.015, 0.87)
UPPER_CARRIER_LEN = 0.104
LOWER_CARRIER_LEN = 0.114


def box_shape(
    size_x: float,
    size_y: float,
    size_z: float,
    center_xyz: tuple[float, float, float],
    *,
    fillet_z: float | None = None,
):
    shape = cq.Workplane("XY").box(size_x, size_y, size_z)
    if fillet_z is not None and fillet_z > 0.0:
        shape = shape.edges("|Z").fillet(fillet_z)
    return shape.translate(center_xyz)


def cyl_y(radius: float, length: float, center_xyz: tuple[float, float, float]):
    x, y, z = center_xyz
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((x, y - (length / 2.0), z))
    )


def cyl_x(radius: float, length: float, center_xyz: tuple[float, float, float]):
    x, y, z = center_xyz
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((x - (length / 2.0), y, z))
    )


def ring_y(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center_xyz: tuple[float, float, float],
):
    return cyl_y(outer_radius, length, center_xyz).cut(
        cyl_y(inner_radius, length + 0.02, center_xyz)
    )


def ring_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center_xyz: tuple[float, float, float],
):
    return cyl_x(outer_radius, length, center_xyz).cut(
        cyl_x(inner_radius, length + 0.02, center_xyz)
    )


def make_tool_pad(size_x: float, size_y: float, size_z: float):
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z)
        .edges("|Z")
        .fillet(0.008)
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-size_x * 0.28, -size_y * 0.24),
                (-size_x * 0.28, size_y * 0.24),
                (size_x * 0.28, -size_y * 0.24),
                (size_x * 0.28, size_y * 0.24),
            ]
        )
        .hole(0.009)
    )


def make_base_structure():
    plate = box_shape(
        BASE_PLATE_X,
        BASE_PLATE_Y,
        BASE_PLATE_Z,
        (0.0, 0.0, SKID_Z + (BASE_PLATE_Z / 2.0)),
        fillet_z=0.018,
    )
    skids = (
        box_shape(SKID_X, SKID_Y, SKID_Z, (-0.17, 0.0, SKID_Z / 2.0), fillet_z=0.01)
        .union(
            box_shape(
                SKID_X,
                SKID_Y,
                SKID_Z,
                (0.17, 0.0, SKID_Z / 2.0),
                fillet_z=0.01,
            )
        )
    )
    mast = box_shape(
        MAST_X,
        MAST_Y,
        MAST_Z,
        (0.0, 0.0, MAST_BASE_Z + (MAST_Z / 2.0)),
        fillet_z=0.012,
    )
    mast_cap = box_shape(
        0.19,
        0.17,
        0.032,
        (0.0, 0.0, MAST_BASE_Z + MAST_Z + 0.016),
        fillet_z=0.008,
    )
    front_panel = box_shape(0.108, 0.008, 0.48, (0.0, 0.074, 0.64), fillet_z=0.003)

    front_gusset = (
        cq.Workplane("XZ")
        .polyline([(-0.05, 0.0), (-0.05, 0.13), (0.05, 0.0)])
        .close()
        .extrude(0.012)
        .translate((0.0, 0.07, MAST_BASE_Z))
    )
    back_gusset = (
        cq.Workplane("XZ")
        .polyline([(-0.05, 0.0), (-0.05, 0.13), (0.05, 0.0)])
        .close()
        .extrude(0.012)
        .translate((0.0, -0.082, MAST_BASE_Z))
    )
    left_gusset = (
        cq.Workplane("YZ")
        .polyline([(-0.05, 0.0), (-0.05, 0.13), (0.05, 0.0)])
        .close()
        .extrude(0.012)
        .translate((-0.082, 0.0, MAST_BASE_Z))
    )
    right_gusset = (
        cq.Workplane("YZ")
        .polyline([(-0.05, 0.0), (-0.05, 0.13), (0.05, 0.0)])
        .close()
        .extrude(0.012)
        .translate((0.07, 0.0, MAST_BASE_Z))
    )

    structure = (
        plate.union(skids)
        .union(mast)
        .union(mast_cap)
        .union(front_panel)
        .union(front_gusset)
        .union(back_gusset)
        .union(left_gusset)
        .union(right_gusset)
    )
    upper_clearance = cyl_y(0.03, 0.16, UPPER_AXIS).union(
        box_shape(0.09, 0.065, 0.14, (UPPER_AXIS[0], 0.102, UPPER_AXIS[2]))
    )
    lower_clearance = cyl_x(0.034, 0.19, LOWER_AXIS).union(
        box_shape(0.13, 0.09, 0.16, (0.11, LOWER_AXIS[1], LOWER_AXIS[2]))
    )
    return structure.cut(upper_clearance).cut(lower_clearance)


def make_upper_carrier():
    axis_x, axis_y, axis_z = UPPER_AXIS
    backplate = box_shape(0.154, 0.028, 0.19, (axis_x, 0.084, axis_z), fillet_z=0.004)
    saddle = box_shape(0.14, 0.064, 0.06, (axis_x, 0.113, axis_z - 0.042), fillet_z=0.004)
    ring = ring_y(0.055, 0.026, UPPER_CARRIER_LEN, UPPER_AXIS).cut(
        box_shape(0.16, 0.12, 0.008, (axis_x, axis_y, axis_z + 0.016))
    )
    cap = box_shape(0.13, 0.042, 0.024, (axis_x, axis_y, axis_z + 0.056), fillet_z=0.003)
    ear_left = box_shape(0.018, 0.05, 0.052, (-0.055, axis_y, axis_z + 0.04), fillet_z=0.002)
    ear_right = box_shape(0.018, 0.05, 0.052, (0.055, axis_y, axis_z + 0.04), fillet_z=0.002)
    gusset_right = (
        cq.Workplane("YZ")
        .polyline([(-0.026, -0.072), (-0.026, -0.012), (0.012, -0.012), (0.054, -0.058)])
        .close()
        .extrude(0.012)
        .translate((0.056, axis_y - 0.018, axis_z))
    )
    gusset_left = (
        cq.Workplane("YZ")
        .polyline([(-0.026, -0.072), (-0.026, -0.012), (0.012, -0.012), (0.054, -0.058)])
        .close()
        .extrude(0.012)
        .translate((-0.068, axis_y - 0.018, axis_z))
    )

    carrier = (
        backplate.union(saddle)
        .union(ring)
        .union(cap)
        .union(ear_left)
        .union(ear_right)
        .union(gusset_left)
        .union(gusset_right)
    )
    shaft_bore = cyl_y(0.028, 0.15, UPPER_AXIS).union(
        box_shape(0.07, 0.05, 0.08, (axis_x, axis_y + 0.01, axis_z + 0.03))
    )
    return carrier.cut(shaft_bore)


def make_lower_carrier():
    axis_x, axis_y, axis_z = LOWER_AXIS
    side_plate = box_shape(0.026, 0.22, 0.22, (0.093, axis_y, axis_z), fillet_z=0.004)
    housing = box_shape(0.07, 0.18, 0.15, (0.118, axis_y, axis_z), fillet_z=0.004)
    drum = ring_x(0.064, 0.031, LOWER_CARRIER_LEN, LOWER_AXIS).cut(
        box_shape(0.13, 0.012, 0.18, (axis_x, axis_y + 0.034, axis_z))
    )
    outer_flange = ring_x(0.078, 0.036, 0.012, (axis_x + 0.063, axis_y, axis_z))
    top_clamp = box_shape(0.022, 0.16, 0.024, (0.156, axis_y, axis_z + 0.074), fillet_z=0.002)
    front_boss = box_shape(0.03, 0.09, 0.07, (0.155, axis_y + 0.07, axis_z), fillet_z=0.002)
    lower_gusset = (
        cq.Workplane("XZ")
        .polyline([(-0.052, -0.102), (-0.052, -0.022), (0.018, -0.058)])
        .close()
        .extrude(0.072)
        .translate((axis_x, axis_y - 0.036, axis_z))
    )

    carrier = (
        side_plate.union(housing)
        .union(drum)
        .union(outer_flange)
        .union(top_clamp)
        .union(front_boss)
        .union(lower_gusset)
    )
    shaft_bore = cyl_x(0.032, 0.17, LOWER_AXIS).union(
        box_shape(0.08, 0.07, 0.08, (axis_x + 0.03, axis_y + 0.02, axis_z))
    )
    return carrier.cut(shaft_bore)


def make_upper_hub():
    shaft = cyl_y(0.022, 0.118, (0.0, 0.0, 0.0))
    rear_collar = cyl_y(0.034, 0.006, (0.0, -0.053, 0.0))
    front_collar = cyl_y(0.034, 0.006, (0.0, 0.053, 0.0))
    neck = box_shape(0.05, 0.036, 0.07, (0.0, 0.078, 0.08), fillet_z=0.002)
    root_block = box_shape(0.078, 0.05, 0.11, (0.0, 0.13, 0.112), fillet_z=0.003)
    cheek_left = box_shape(0.014, 0.05, 0.118, (-0.028, 0.13, 0.112), fillet_z=0.002)
    cheek_right = box_shape(0.014, 0.05, 0.118, (0.028, 0.13, 0.112), fillet_z=0.002)

    return (
        shaft.union(rear_collar)
        .union(front_collar)
        .union(neck)
        .union(root_block)
        .union(cheek_left)
        .union(cheek_right)
    )


def make_upper_arm():
    riser = box_shape(0.068, 0.05, 0.3, (0.0, 0.13, 0.19), fillet_z=0.004)
    boom = box_shape(0.058, 0.36, 0.048, (0.0, 0.32, 0.34), fillet_z=0.004)
    top_rail = box_shape(0.036, 0.28, 0.034, (0.0, 0.29, 0.374), fillet_z=0.003)
    end_plate = box_shape(0.088, 0.02, 0.11, (0.0, 0.53, 0.34), fillet_z=0.002)
    gusset_right = (
        cq.Workplane("YZ")
        .polyline([(0.12, 0.07), (0.12, 0.29), (0.3, 0.29)])
        .close()
        .extrude(0.01)
        .translate((0.026, 0.0, 0.0))
    )
    gusset_left = (
        cq.Workplane("YZ")
        .polyline([(0.12, 0.07), (0.12, 0.29), (0.3, 0.29)])
        .close()
        .extrude(0.01)
        .translate((-0.036, 0.0, 0.0))
    )

    return riser.union(boom).union(top_rail).union(end_plate).union(gusset_left).union(gusset_right)


def make_upper_pad():
    mount = box_shape(0.052, 0.034, 0.05, (0.0, 0.552, 0.34), fillet_z=0.002)
    pad = make_tool_pad(0.11, 0.085, 0.024).translate((0.0, 0.61, 0.34))
    return mount.union(pad)


def make_lower_hub():
    shaft = cyl_x(0.026, 0.12, (0.0, 0.0, 0.0))
    inner_collar = cyl_x(0.038, 0.007, (-0.0565, 0.0, 0.0))
    outer_collar = cyl_x(0.038, 0.007, (0.0565, 0.0, 0.0))
    neck = box_shape(0.07, 0.036, 0.08, (0.11, 0.035, 0.0), fillet_z=0.002)
    root_socket = box_shape(0.074, 0.11, 0.12, (0.198, 0.085, 0.0), fillet_z=0.003)
    clamp_fin = box_shape(0.018, 0.09, 0.08, (0.16, 0.055, 0.0), fillet_z=0.002)
    tie_plate = box_shape(0.04, 0.05, 0.1, (0.15, 0.03, 0.0), fillet_z=0.002)

    return (
        shaft.union(inner_collar)
        .union(outer_collar)
        .union(neck)
        .union(root_socket)
        .union(clamp_fin)
        .union(tie_plate)
    )


def make_lower_arm():
    root_spine = box_shape(0.04, 0.14, 0.08, (0.172, 0.16, 0.0), fillet_z=0.002)
    beam = box_shape(0.075, 0.56, 0.06, (0.215, 0.41, 0.0), fillet_z=0.004)
    top_rail = box_shape(0.038, 0.48, 0.038, (0.215, 0.38, 0.05), fillet_z=0.003)
    underside_rib = box_shape(0.03, 0.34, 0.038, (0.218, 0.29, -0.042), fillet_z=0.002)
    end_plate = box_shape(0.096, 0.022, 0.12, (0.215, 0.71, 0.0), fillet_z=0.002)
    pad_link = box_shape(0.038, 0.06, 0.058, (0.215, 0.732, 0.0), fillet_z=0.002)
    gusset_right = (
        cq.Workplane("YZ")
        .polyline([(0.14, -0.055), (0.14, -0.012), (0.34, -0.028)])
        .close()
        .extrude(0.012)
        .translate((0.19, 0.0, 0.0))
    )
    gusset_left = (
        cq.Workplane("YZ")
        .polyline([(0.14, -0.055), (0.14, -0.012), (0.34, -0.028)])
        .close()
        .extrude(0.012)
        .translate((0.228, 0.0, 0.0))
    )

    return (
        root_spine.union(beam)
        .union(top_rail)
        .union(underside_rib)
        .union(end_plate)
        .union(pad_link)
        .union(gusset_left)
        .union(gusset_right)
    )


def make_lower_pad():
    mount = box_shape(0.052, 0.034, 0.05, (0.215, 0.76, 0.0), fillet_z=0.002)
    pad = make_tool_pad(0.12, 0.095, 0.026).translate((0.215, 0.82, 0.0))
    return mount.union(pad)


def make_upper_carrier_visual():
    axis_x, axis_y, axis_z = UPPER_AXIS
    backplate = box_shape(0.154, 0.026, 0.22, (axis_x, 0.083, axis_z), fillet_z=0.004)
    bearing = ring_y(0.05, 0.026, 0.044, (axis_x, 0.104, axis_z))
    cap = box_shape(0.124, 0.024, 0.028, (axis_x, 0.114, axis_z + 0.068), fillet_z=0.003)
    left_rib = box_shape(0.018, 0.044, 0.11, (-0.054, 0.096, axis_z + 0.01), fillet_z=0.002)
    right_rib = box_shape(0.018, 0.044, 0.11, (0.054, 0.096, axis_z + 0.01), fillet_z=0.002)
    lower_saddle = box_shape(0.14, 0.03, 0.05, (axis_x, 0.106, axis_z - 0.064), fillet_z=0.003)
    return backplate.union(bearing).union(cap).union(left_rib).union(right_rib).union(lower_saddle)


def make_lower_carrier_visual():
    axis_x, axis_y, axis_z = LOWER_AXIS
    side_plate = box_shape(0.022, 0.22, 0.22, (0.091, axis_y, axis_z), fillet_z=0.004)
    drum = ring_x(0.056, 0.03, 0.04, (0.116, axis_y, axis_z))
    outer_block = box_shape(0.04, 0.16, 0.14, (0.116, axis_y, axis_z), fillet_z=0.003)
    top_clamp = box_shape(0.018, 0.16, 0.026, (0.127, axis_y, axis_z + 0.078), fillet_z=0.002)
    front_boss = box_shape(0.018, 0.08, 0.08, (0.127, axis_y + 0.07, axis_z), fillet_z=0.002)
    return side_plate.union(drum).union(outer_block).union(top_clamp).union(front_boss)


def make_upper_hub_visual():
    contact_shoe = box_shape(0.076, 0.028, 0.11, (0.0, 0.014, 0.08), fillet_z=0.002)
    neck = box_shape(0.042, 0.22, 0.07, (0.0, 0.12, 0.14), fillet_z=0.002)
    root = box_shape(0.078, 0.12, 0.12, (0.0, 0.23, 0.20), fillet_z=0.003)
    cheeks = (
        box_shape(0.014, 0.12, 0.12, (-0.029, 0.23, 0.20), fillet_z=0.002)
        .union(box_shape(0.014, 0.12, 0.12, (0.029, 0.23, 0.20), fillet_z=0.002))
    )
    return contact_shoe.union(neck).union(root).union(cheeks)


def make_upper_arm_visual():
    root_block = box_shape(0.06, 0.16, 0.09, (0.0, 0.31, 0.26), fillet_z=0.003)
    spine = box_shape(0.034, 0.2, 0.06, (0.0, 0.43, 0.36), fillet_z=0.003)
    boom = box_shape(0.055, 0.24, 0.05, (0.0, 0.54, 0.40), fillet_z=0.003)
    return root_block.union(spine).union(boom)


def make_upper_pad_visual():
    mount = box_shape(0.05, 0.085, 0.05, (0.0, 0.66, 0.40), fillet_z=0.002)
    pad = make_tool_pad(0.11, 0.085, 0.024).translate((0.0, 0.75, 0.40))
    return mount.union(pad)


def make_lower_hub_visual():
    contact_shoe = box_shape(0.03, 0.11, 0.12, (0.015, 0.06, 0.0), fillet_z=0.002)
    neck = box_shape(0.12, 0.05, 0.08, (0.09, 0.05, 0.0), fillet_z=0.002)
    root = box_shape(0.1, 0.12, 0.12, (0.21, 0.12, 0.0), fillet_z=0.003)
    clamp = box_shape(0.018, 0.09, 0.08, (0.155, 0.055, 0.0), fillet_z=0.002)
    return contact_shoe.union(neck).union(root).union(clamp)


def make_lower_arm_visual():
    root_block = box_shape(0.08, 0.16, 0.08, (0.24, 0.20, 0.0), fillet_z=0.003)
    beam = box_shape(0.11, 0.56, 0.06, (0.29, 0.48, 0.0), fillet_z=0.004)
    top_rail = box_shape(0.05, 0.44, 0.04, (0.29, 0.44, 0.05), fillet_z=0.003)
    end_plate = box_shape(0.09, 0.03, 0.12, (0.29, 0.77, 0.0), fillet_z=0.002)
    return root_block.union(beam).union(top_rail).union(end_plate)


def make_lower_pad_visual():
    mount = box_shape(0.05, 0.07, 0.05, (0.29, 0.81, -0.01), fillet_z=0.002)
    pad = make_tool_pad(0.12, 0.095, 0.026).translate((0.29, 0.89, -0.01))
    return mount.union(pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_arm_positioning_tree")

    steel = model.material("steel", color=(0.45, 0.47, 0.5))
    housing_dark = model.material("housing_dark", color=(0.2, 0.21, 0.23))
    arm_dark = model.material("arm_dark", color=(0.27, 0.29, 0.31))
    pad_finish = model.material("pad_finish", color=(0.71, 0.73, 0.76))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(make_base_structure(), "mast_structure"),
        material=steel,
        name="base_structure",
    )
    mast.visual(
        mesh_from_cadquery(make_upper_carrier_visual(), "upper_carrier"),
        material=housing_dark,
        name="upper_carrier",
    )
    mast.visual(
        mesh_from_cadquery(make_lower_carrier_visual(), "lower_carrier"),
        material=housing_dark,
        name="lower_carrier",
    )

    upper_branch = model.part("upper_branch")
    upper_branch.visual(
        mesh_from_cadquery(make_upper_hub_visual(), "upper_hub"),
        material=housing_dark,
        name="upper_hub",
    )
    upper_branch.visual(
        mesh_from_cadquery(make_upper_arm_visual(), "upper_arm"),
        material=arm_dark,
        name="upper_arm",
    )
    upper_branch.visual(
        mesh_from_cadquery(make_upper_pad_visual(), "upper_pad"),
        material=pad_finish,
        name="upper_pad",
    )

    lower_branch = model.part("lower_branch")
    lower_branch.visual(
        mesh_from_cadquery(make_lower_hub_visual(), "lower_hub"),
        material=housing_dark,
        name="lower_hub",
    )
    lower_branch.visual(
        mesh_from_cadquery(make_lower_arm_visual(), "lower_arm"),
        material=arm_dark,
        name="lower_arm",
    )
    lower_branch.visual(
        mesh_from_cadquery(make_lower_pad_visual(), "lower_pad"),
        material=pad_finish,
        name="lower_pad",
    )

    model.articulation(
        "mast_to_upper_branch",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=upper_branch,
        origin=Origin(xyz=UPPER_AXIS),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.0, lower=-0.7, upper=0.7),
    )
    model.articulation(
        "mast_to_lower_branch",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=lower_branch,
        origin=Origin(xyz=LOWER_AXIS),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.9, lower=-0.8, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    upper_branch = object_model.get_part("upper_branch")
    lower_branch = object_model.get_part("lower_branch")
    upper_joint = object_model.get_articulation("mast_to_upper_branch")
    lower_joint = object_model.get_articulation("mast_to_lower_branch")

    base_structure = mast.get_visual("base_structure")
    upper_carrier = mast.get_visual("upper_carrier")
    lower_carrier = mast.get_visual("lower_carrier")
    upper_hub = upper_branch.get_visual("upper_hub")
    upper_arm = upper_branch.get_visual("upper_arm")
    upper_pad = upper_branch.get_visual("upper_pad")
    lower_hub = lower_branch.get_visual("lower_hub")
    lower_arm = lower_branch.get_visual("lower_arm")
    lower_pad = lower_branch.get_visual("lower_pad")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.expect_contact(
        upper_branch,
        mast,
        elem_a=upper_hub,
        elem_b=upper_carrier,
        contact_tol=0.002,
        name="upper_branch_is_carried_by_front_hub",
    )
    ctx.expect_contact(
        lower_branch,
        mast,
        elem_a=lower_hub,
        elem_b=lower_carrier,
        contact_tol=0.002,
        name="lower_branch_is_carried_by_side_hub",
    )

    ctx.expect_origin_gap(
        upper_branch,
        mast,
        axis="y",
        min_gap=0.11,
        max_gap=0.14,
        name="upper_joint_sits_forward_of_mast_centerline",
    )
    ctx.expect_origin_gap(
        lower_branch,
        mast,
        axis="x",
        min_gap=0.12,
        max_gap=0.15,
        name="lower_joint_sits_outboard_of_mast",
    )

    ctx.expect_gap(
        upper_branch,
        mast,
        axis="y",
        positive_elem=upper_pad,
        negative_elem=base_structure,
        min_gap=0.35,
        name="upper_tool_pad_projects_forward",
    )
    ctx.expect_gap(
        lower_branch,
        mast,
        axis="y",
        positive_elem=lower_pad,
        negative_elem=base_structure,
        min_gap=0.45,
        name="lower_tool_pad_projects_far_forward",
    )

    with ctx.pose({upper_joint: 0.65}):
        ctx.expect_gap(
            upper_branch,
            mast,
            axis="y",
            positive_elem=upper_arm,
            negative_elem=base_structure,
            min_gap=0.10,
            name="upper_arm_clears_mast_when_swung",
        )

    with ctx.pose({lower_joint: 0.55}):
        ctx.expect_gap(
            lower_branch,
            mast,
            axis="x",
            positive_elem=lower_arm,
            negative_elem=base_structure,
            min_gap=0.012,
            name="lower_arm_stays_outboard_of_mast",
        )

    with ctx.pose({upper_joint: -0.65, lower_joint: 0.55}):
        ctx.expect_gap(
            upper_branch,
            lower_branch,
            axis="z",
            positive_elem=upper_pad,
            negative_elem=lower_pad,
            min_gap=0.16,
            name="tool_pads_keep_vertical_clearance_in_near_worst_pose",
        )

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="articulation_sweep_stays_clear",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
