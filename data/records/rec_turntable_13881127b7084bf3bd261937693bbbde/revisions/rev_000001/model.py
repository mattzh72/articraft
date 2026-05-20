from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = model.material("walnut", rgba=(0.36, 0.22, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.04, 0.04, 0.04, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.18, 0.19, 1.0))
    label_gray = model.material("label_gray", rgba=(0.70, 0.71, 0.73, 1.0))

    plinth = model.part("plinth")
    plinth_shell = _mesh(
        "plinth_shell",
        ExtrudeGeometry(rounded_rect_profile(0.45, 0.36, 0.018, corner_segments=8), 0.06),
    )
    plinth.visual(
        plinth_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=walnut,
        name="plinth_shell",
    )
    plinth.visual(
        Box((0.428, 0.338, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0615)),
        material=dark_gray,
        name="top_deck",
    )
    for foot_name, x_pos, y_pos in (
        ("foot_front_left", -0.17, 0.12),
        ("foot_front_right", 0.17, 0.12),
        ("foot_rear_left", -0.17, -0.12),
        ("foot_rear_right", 0.17, -0.12),
    ):
        plinth.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(x_pos, y_pos, -0.006)),
            material=rubber_black,
            name=foot_name,
        )
    plinth.visual(
        Cylinder(radius=0.068, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=dark_gray,
        name="platter_support",
    )
    plinth.visual(
        Box((0.030, 0.012, 0.004)),
        origin=Origin(xyz=(-0.170, 0.120, 0.063)),
        material=brushed_aluminum,
        name="speed_switch",
    )
    plinth.visual(
        Cylinder(radius=0.042, length=0.008),
        origin=Origin(xyz=(0.155, -0.105, 0.064)),
        material=dark_gray,
        name="tonearm_mount_flange",
    )
    plinth.visual(
        Cylinder(radius=0.030, length=0.036),
        origin=Origin(xyz=(0.155, -0.105, 0.078)),
        material=dark_gray,
        name="tonearm_base_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.005, length=0.024),
        origin=Origin(xyz=(0.192, -0.015, 0.072)),
        material=brushed_aluminum,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.020, 0.006, 0.004)),
        origin=Origin(xyz=(0.192, -0.015, 0.086)),
        material=satin_black,
        name="arm_rest_cradle",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.45, 0.36, 0.09)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.152, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed_aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.146, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=rubber_black,
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.149, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0209)),
        material=satin_black,
        name="record_disc",
    )
    platter.visual(
        Cylinder(radius=0.050, length=0.0006),
        origin=Origin(xyz=(0.0, 0.0, 0.0221)),
        material=label_gray,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=brushed_aluminum,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.152, length=0.032),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    tonearm = model.part("tonearm")
    tonearm_tube = _mesh(
        "tonearm_tube",
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.020),
                (0.004, 0.055, 0.021),
                (0.008, 0.160, 0.017),
                (0.006, 0.230, 0.013),
            ],
            radius=0.005,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    tonearm.visual(
        Cylinder(radius=0.014, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=brushed_aluminum,
        name="pivot_housing",
    )
    tonearm.visual(
        tonearm_tube,
        material=brushed_aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.045),
        origin=Origin(xyz=(0.0, -0.0225, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="rear_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.0135, length=0.030),
        origin=Origin(xyz=(0.0, -0.060, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.014, 0.028, 0.005)),
        origin=Origin(xyz=(0.006, 0.237, 0.013)),
        material=satin_black,
        name="headshell",
    )
    tonearm.visual(
        Box((0.018, 0.014, 0.007)),
        origin=Origin(xyz=(0.006, 0.247, 0.008)),
        material=satin_black,
        name="cartridge_body",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.04, 0.33, 0.05)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.10, 0.018)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    model.articulation(
        "tonearm_pivot",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.155, -0.105, 0.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=1.2, lower=-0.20, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm = object_model.get_part("tonearm")
    tonearm_pivot = object_model.get_articulation("tonearm_pivot")

    ctx.expect_origin_distance(
        platter,
        plinth,
        axes="xy",
        max_dist=0.001,
        name="platter stays centered on plinth axis",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        negative_elem="platter_support",
        max_gap=0.0005,
        max_penetration=0.0,
        name="platter sits on central support without sinking",
    )
    with ctx.pose({platter_spin: 1.57}):
        ctx.expect_origin_distance(
            platter,
            plinth,
            axes="xy",
            max_dist=0.001,
            name="rotated platter remains centered on main axis",
        )
    with ctx.pose({tonearm_pivot: 0.95}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="cartridge_body",
            elem_b="record_disc",
            min_overlap=0.010,
            name="tonearm can swing cartridge over the record",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge_body",
            negative_elem="record_disc",
            min_gap=0.0005,
            max_gap=0.010,
            name="cartridge rides just above the record surface",
        )

    rest_cartridge = ctx.part_element_world_aabb(tonearm, elem="cartridge_body")
    with ctx.pose({tonearm_pivot: 0.95}):
        swung_cartridge = ctx.part_element_world_aabb(tonearm, elem="cartridge_body")

    def _center_x(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    rest_x = _center_x(rest_cartridge)
    swung_x = _center_x(swung_cartridge)
    ctx.check(
        "positive tonearm motion swings inward",
        rest_x is not None and swung_x is not None and swung_x < rest_x - 0.10,
        details=f"rest_x={rest_x}, swung_x={swung_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
