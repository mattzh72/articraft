from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Cylinder primitive aligned along the binocular optical axis (+X)."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Cylinder primitive aligned along the left-right axis (+Y)."""
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _tube_mesh(name: str, outer_r: float, inner_r: float, length: float):
    """Thin-walled hollow tube, authored on local Z then rotated to local X."""
    geom = LatheGeometry.from_shell_profiles(
        outer_profile=((outer_r, -length / 2.0), (outer_r, length / 2.0)),
        inner_profile=((inner_r, -length / 2.0), (inner_r, length / 2.0)),
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_8x21_theatre_binocular")

    satin_black = model.material("satin_black", rgba=(0.005, 0.005, 0.006, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    graphite = model.material("dark_graphite", rgba=(0.10, 0.105, 0.11, 1.0))
    glass = model.material("blue_coated_glass", rgba=(0.05, 0.18, 0.30, 0.62))
    hinge_metal = model.material("brushed_black_metal", rgba=(0.025, 0.025, 0.028, 1.0))

    # Object frame: +X looks toward the objectives, -X toward the eyepieces,
    # +Y is the second barrel side, and Z is vertical.
    tube_y = 0.032
    tube_radius = 0.0128

    barrel_0 = model.part("barrel_0")
    barrel_1 = model.part("barrel_1")

    def add_barrel(part, y: float, suffix: str) -> None:
        part.visual(
            Cylinder(radius=tube_radius, length=0.074),
            origin=_x_cylinder_origin(0.002, y, 0.0),
            material=satin_black,
            name=f"main_tube_{suffix}",
        )
        part.visual(
            _tube_mesh(f"objective_bezel_{suffix}", 0.0152, 0.0106, 0.014),
            origin=Origin(xyz=(0.045, y, 0.0)),
            material=graphite,
            name=f"objective_bezel_{suffix}",
        )
        part.visual(
            Cylinder(radius=0.0109, length=0.0020),
            origin=_x_cylinder_origin(0.0530, y, 0.0),
            material=glass,
            name=f"objective_lens_{suffix}",
        )
        part.visual(
            _tube_mesh(f"ocular_socket_{suffix}", 0.0108, 0.0077, 0.018),
            origin=Origin(xyz=(-0.043, y, 0.0)),
            material=graphite,
            name=f"ocular_socket_{suffix}",
        )
        part.visual(
            Cylinder(radius=0.0079, length=0.0015),
            origin=_x_cylinder_origin(-0.05275, y, 0.0),
            material=glass,
            name=f"ocular_lens_{suffix}",
        )
        # Slim longitudinal rubber grip strips make the housings read as
        # compact theatre binocular barrels rather than simple pipes.
        part.visual(
            Box((0.050, 0.0040, 0.0035)),
            origin=Origin(xyz=(0.004, y, 0.0135)),
            material=rubber,
            name=f"top_grip_{suffix}",
        )
        part.visual(
            Box((0.048, 0.0035, 0.012)),
            origin=Origin(xyz=(0.004, y * 0.985, -0.0005)),
            material=rubber,
            name=f"side_grip_{suffix}",
        )

    add_barrel(barrel_0, -tube_y, "0")
    add_barrel(barrel_1, tube_y, "1")

    # Half-bridges visibly tie each optical barrel into the central folding hinge.
    barrel_0.visual(
        Box((0.074, 0.024, 0.009)),
        origin=Origin(xyz=(-0.002, -0.014, 0.0)),
        material=hinge_metal,
        name="bridge_leaf_0",
    )
    barrel_1.visual(
        Box((0.074, 0.024, 0.009)),
        origin=Origin(xyz=(-0.002, 0.014, 0.0)),
        material=hinge_metal,
        name="bridge_leaf_1",
    )
    barrel_0.visual(
        Cylinder(radius=0.0048, length=0.030),
        origin=_x_cylinder_origin(-0.026, -0.0010, 0.0),
        material=hinge_metal,
        name="hinge_barrel_0",
    )
    barrel_0.visual(
        Cylinder(radius=0.0048, length=0.030),
        origin=_x_cylinder_origin(0.026, -0.0010, 0.0),
        material=hinge_metal,
        name="hinge_knuckle_0",
    )
    barrel_1.visual(
        Cylinder(radius=0.0048, length=0.020),
        origin=_x_cylinder_origin(0.0, 0.0010, 0.0),
        material=hinge_metal,
        name="hinge_barrel_1",
    )

    # A small top bridge and exposed shaft carry the center focus wheel.
    barrel_0.visual(
        Box((0.036, 0.016, 0.008)),
        origin=Origin(xyz=(-0.028, -0.004, 0.016)),
        material=hinge_metal,
        name="focus_pedestal",
    )
    barrel_0.visual(
        Box((0.030, 0.010, 0.012)),
        origin=Origin(xyz=(-0.028, -0.006, 0.009)),
        material=hinge_metal,
        name="focus_support",
    )
    barrel_0.visual(
        Box((0.007, 0.004, 0.022)),
        origin=Origin(xyz=(-0.030, -0.011, 0.024)),
        material=hinge_metal,
        name="focus_fork",
    )
    barrel_0.visual(
        Cylinder(radius=0.0030, length=0.026),
        origin=_y_cylinder_origin(-0.030, 0.0, 0.034),
        material=hinge_metal,
        name="focus_axle",
    )

    center_knob = model.part("center_knob")
    focus_geometry = KnobGeometry(
        0.024,
        0.018,
        body_style="hourglass",
        base_diameter=0.022,
        top_diameter=0.022,
        edge_radius=0.0008,
        grip=KnobGrip(style="ribbed", count=28, depth=0.0008, width=0.0012),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        bore=KnobBore(style="round", diameter=0.0075),
    )
    center_knob.visual(
        mesh_from_geometry(focus_geometry, "center_focus_knob"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="ribbed_wheel",
    )

    eyecup_0 = model.part("eyecup_0")
    eyecup_1 = model.part("eyecup_1")
    eyecup_mesh_0 = _tube_mesh("twist_eyecup_0", 0.0128, 0.01078, 0.017)
    eyecup_mesh_1 = _tube_mesh("twist_eyecup_1", 0.0128, 0.01078, 0.017)
    lip_mesh_0 = _tube_mesh("eye_lip_0", 0.0135, 0.0088, 0.0030)
    lip_mesh_1 = _tube_mesh("eye_lip_1", 0.0135, 0.0088, 0.0030)
    for cup, mesh, lip_mesh, suffix in (
        (eyecup_0, eyecup_mesh_0, lip_mesh_0, "0"),
        (eyecup_1, eyecup_mesh_1, lip_mesh_1, "1"),
    ):
        cup.visual(
            mesh,
            origin=Origin(xyz=(-0.0065, 0.0, 0.0)),
            material=rubber,
            name=f"twist_sleeve_{suffix}",
        )
        cup.visual(
            lip_mesh,
            origin=Origin(xyz=(-0.0165, 0.0, 0.0)),
            material=rubber,
            name=f"eye_lip_{suffix}",
        )

    model.articulation(
        "central_hinge",
        ArticulationType.REVOLUTE,
        parent=barrel_0,
        child=barrel_1,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "focus_knob_joint",
        ArticulationType.REVOLUTE,
        parent=barrel_0,
        child=center_knob,
        origin=Origin(xyz=(-0.030, 0.0, 0.034)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "eyecup_0_twist",
        ArticulationType.REVOLUTE,
        parent=barrel_0,
        child=eyecup_0,
        origin=Origin(xyz=(-0.052, -tube_y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=0.0, upper=math.radians(105.0)),
    )
    model.articulation(
        "eyecup_1_twist",
        ArticulationType.REVOLUTE,
        parent=barrel_1,
        child=eyecup_1,
        origin=Origin(xyz=(-0.052, tube_y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=0.0, upper=math.radians(105.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel_0 = object_model.get_part("barrel_0")
    barrel_1 = object_model.get_part("barrel_1")
    center_knob = object_model.get_part("center_knob")
    eyecup_0 = object_model.get_part("eyecup_0")
    eyecup_1 = object_model.get_part("eyecup_1")
    hinge = object_model.get_articulation("central_hinge")
    focus = object_model.get_articulation("focus_knob_joint")
    cup0 = object_model.get_articulation("eyecup_0_twist")
    cup1 = object_model.get_articulation("eyecup_1_twist")

    ctx.expect_overlap(
        barrel_0,
        barrel_1,
        axes="yz",
        min_overlap=0.006,
        elem_a="hinge_barrel_0",
        elem_b="hinge_barrel_1",
        name="two barrel housings share the central hinge axis",
    )
    ctx.expect_gap(
        center_knob,
        barrel_0,
        axis="z",
        min_gap=-0.001,
        max_gap=0.004,
        positive_elem="ribbed_wheel",
        negative_elem="focus_pedestal",
        name="focus wheel is seated at the bridge pedestal",
    )
    ctx.expect_overlap(
        eyecup_0,
        barrel_0,
        axes="yz",
        min_overlap=0.010,
        elem_a="twist_sleeve_0",
        elem_b="ocular_socket_0",
        name="eyecup 0 is coaxial with its eyepiece",
    )
    ctx.expect_overlap(
        eyecup_1,
        barrel_1,
        axes="yz",
        min_overlap=0.010,
        elem_a="twist_sleeve_1",
        elem_b="ocular_socket_1",
        name="eyecup 1 is coaxial with its eyepiece",
    )

    def _aabb_center_z(bounds):
        if bounds is None:
            return None
        return (bounds[0][2] + bounds[1][2]) * 0.5

    rest_z = _aabb_center_z(ctx.part_element_world_aabb(barrel_1, elem="objective_lens_1"))
    with ctx.pose({hinge: 0.15, focus: 1.0, cup0: math.radians(90.0), cup1: math.radians(90.0)}):
        folded_z = _aabb_center_z(ctx.part_element_world_aabb(barrel_1, elem="objective_lens_1"))
        ctx.expect_overlap(
            eyecup_0,
            barrel_0,
            axes="x",
            min_overlap=0.001,
            elem_a="twist_sleeve_0",
            elem_b="ocular_socket_0",
            name="eyecup 0 remains seated while twisted",
        )
        ctx.expect_overlap(
            eyecup_1,
            barrel_1,
            axes="x",
            min_overlap=0.001,
            elem_a="twist_sleeve_1",
            elem_b="ocular_socket_1",
            name="eyecup 1 remains seated while twisted",
        )

    ctx.check(
        "central hinge changes barrel angle",
        rest_z is not None and folded_z is not None and abs(folded_z - rest_z) > 0.003,
        details=f"rest_z={rest_z}, folded_z={folded_z}",
    )

    return ctx.report()


object_model = build_object_model()
