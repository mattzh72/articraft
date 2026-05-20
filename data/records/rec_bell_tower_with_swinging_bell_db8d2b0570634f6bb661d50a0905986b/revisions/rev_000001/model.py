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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthodox_onion_dome_bell_tower")

    plaster = model.material("plaster", rgba=(0.94, 0.93, 0.90, 1.0))
    stone = model.material("stone", rgba=(0.82, 0.80, 0.76, 1.0))
    patinated_copper = model.material("patinated_copper", rgba=(0.28, 0.53, 0.42, 1.0))
    gilded_metal = model.material("gilded_metal", rgba=(0.80, 0.68, 0.24, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.28, 0.20, 0.12, 1.0))
    timber = model.material("timber", rgba=(0.36, 0.24, 0.16, 1.0))

    dome_profile = [
        (0.00, 0.00),
        (0.48, 0.00),
        (0.46, 0.10),
        (0.39, 0.28),
        (0.29, 0.56),
        (0.25, 0.82),
        (0.34, 1.10),
        (0.28, 1.34),
        (0.18, 1.54),
        (0.08, 1.68),
        (0.03, 1.76),
        (0.00, 1.80),
    ]
    dome_mesh = mesh_from_geometry(
        LatheGeometry(dome_profile, segments=72),
        "tower_onion_dome",
    )

    bell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.026, -0.20),
                (0.072, -0.24),
                (0.132, -0.35),
                (0.220, -0.51),
                (0.300, -0.67),
                (0.344, -0.79),
                (0.362, -0.82),
            ],
            [
                (0.008, -0.22),
                (0.030, -0.27),
                (0.080, -0.38),
                (0.154, -0.53),
                (0.246, -0.68),
                (0.314, -0.79),
                (0.330, -0.82),
            ],
            segments=64,
            start_cap="round",
            end_cap="flat",
            lip_samples=8,
        ),
        "tower_bell_shell",
    )

    tower = model.part("tower_body")
    tower.visual(
        Cylinder(radius=1.05, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=stone,
        name="plinth",
    )
    tower.visual(
        Cylinder(radius=0.86, length=4.50),
        origin=Origin(xyz=(0.0, 0.0, 2.67)),
        material=plaster,
        name="shaft",
    )
    tower.visual(
        Cylinder(radius=0.94, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 4.92)),
        material=stone,
        name="belfry_base_ring",
    )

    column_radius = 0.13
    column_height = 1.18
    column_z = 5.63
    column_ring_radius = 0.67
    front_left_angle = math.radians(55.0)
    left_rear_angle = math.radians(115.0)
    rear_angle = math.radians(180.0)
    right_rear_angle = math.radians(245.0)
    front_right_angle = math.radians(305.0)
    front_right_inner_angle = math.radians(-55.0)

    tower.visual(
        Cylinder(radius=column_radius, length=column_height),
        origin=Origin(
            xyz=(
                column_ring_radius * math.cos(front_left_angle),
                column_ring_radius * math.sin(front_left_angle),
                column_z,
            )
        ),
        material=plaster,
        name="front_left_pier",
    )
    tower.visual(
        Cylinder(radius=column_radius, length=column_height),
        origin=Origin(
            xyz=(
                column_ring_radius * math.cos(left_rear_angle),
                column_ring_radius * math.sin(left_rear_angle),
                column_z,
            )
        ),
        material=plaster,
        name="left_rear_pier",
    )
    tower.visual(
        Cylinder(radius=column_radius, length=column_height),
        origin=Origin(
            xyz=(
                column_ring_radius * math.cos(rear_angle),
                column_ring_radius * math.sin(rear_angle),
                column_z,
            )
        ),
        material=plaster,
        name="rear_pier",
    )
    tower.visual(
        Cylinder(radius=column_radius, length=column_height),
        origin=Origin(
            xyz=(
                column_ring_radius * math.cos(right_rear_angle),
                column_ring_radius * math.sin(right_rear_angle),
                column_z,
            )
        ),
        material=plaster,
        name="right_rear_pier",
    )
    tower.visual(
        Cylinder(radius=column_radius, length=column_height),
        origin=Origin(
            xyz=(
                column_ring_radius * math.cos(front_right_angle),
                column_ring_radius * math.sin(front_right_angle),
                column_z,
            )
        ),
        material=plaster,
        name="front_right_pier",
    )
    tower.visual(
        Cylinder(radius=column_radius, length=column_height),
        origin=Origin(
            xyz=(
                column_ring_radius * math.cos(front_right_inner_angle),
                column_ring_radius * math.sin(front_right_inner_angle),
                column_z,
            )
        ),
        material=plaster,
        name="front_right_pier_mirror",
    )

    tower.visual(
        Box((0.18, 1.18, 0.16)),
        origin=Origin(xyz=(0.40, 0.0, 6.02)),
        material=timber,
        name="headstock_beam",
    )
    tower.visual(
        Cylinder(radius=0.91, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 6.34)),
        material=stone,
        name="belfry_top_ring",
    )
    tower.visual(
        Cylinder(radius=0.48, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 6.61)),
        material=stone,
        name="dome_drum",
    )
    tower.visual(
        dome_mesh,
        origin=Origin(xyz=(0.0, 0.0, 6.76)),
        material=patinated_copper,
        name="onion_dome",
    )
    tower.visual(
        Cylinder(radius=0.035, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 8.73)),
        material=gilded_metal,
        name="spire",
    )
    tower.visual(
        Cylinder(radius=0.075, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 8.96)),
        material=gilded_metal,
        name="orb",
    )
    tower.visual(
        Box((0.038, 0.038, 0.44)),
        origin=Origin(xyz=(0.0, 0.0, 9.24)),
        material=gilded_metal,
        name="cross_upright",
    )
    tower.visual(
        Box((0.17, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 9.36)),
        material=gilded_metal,
        name="cross_upper_bar",
    )
    tower.visual(
        Box((0.34, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 9.24)),
        material=gilded_metal,
        name="cross_main_bar",
    )
    tower.visual(
        Box((0.26, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 9.10), rpy=(0.0, -0.28, 0.0)),
        material=gilded_metal,
        name="cross_foot_bar",
    )
    tower.inertial = Inertial.from_geometry(
        Box((2.10, 2.10, 9.10)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 4.55)),
    )

    bell = model.part("bell")
    bell.visual(
        Box((0.07, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=timber,
        name="bell_headstock",
    )
    bell.visual(
        Box((0.032, 0.030, 0.18)),
        origin=Origin(xyz=(0.0, 0.10, -0.22)),
        material=timber,
        name="left_hanger",
    )
    bell.visual(
        Box((0.032, 0.030, 0.18)),
        origin=Origin(xyz=(0.0, -0.10, -0.22)),
        material=timber,
        name="right_hanger",
    )
    bell.visual(
        Box((0.08, 0.24, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.31)),
        material=timber,
        name="crown_yoke",
    )
    bell.visual(
        Cylinder(radius=0.055, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.26)),
        material=dark_bronze,
        name="crown_stem",
    )
    bell.visual(
        Cylinder(radius=0.070, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.37)),
        material=dark_bronze,
        name="crown_block",
    )
    bell.visual(
        bell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material=dark_bronze,
        name="bell_shell",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.39, length=0.98),
        mass=460.0,
        origin=Origin(xyz=(0.0, 0.0, -0.49)),
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.40, 0.0, 6.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=1.3,
            lower=-0.60,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower_body")
    bell = object_model.get_part("bell")
    swing = object_model.get_articulation("bell_swing")
    limits = swing.motion_limits

    ctx.check(
        "bell swing joint uses a horizontal headstock axis",
        swing.axis == (0.0, -1.0, 0.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"axis={swing.axis}, limits={limits}",
    )

    with ctx.pose({swing: 0.0}):
        ctx.expect_contact(
            tower,
            bell,
            elem_a="headstock_beam",
            elem_b="bell_headstock",
            name="bell headstock is carried by the beam",
        )
        ctx.expect_gap(
            tower,
            bell,
            axis="z",
            positive_elem="headstock_beam",
            negative_elem="bell_shell",
            min_gap=0.12,
            max_gap=0.36,
            name="bell crown hangs below the headstock beam",
        )
        ctx.expect_gap(
            bell,
            tower,
            axis="z",
            positive_elem="bell_shell",
            negative_elem="belfry_base_ring",
            min_gap=0.12,
            max_gap=0.30,
            name="bell mouth clears the belfry sill ring",
        )
        ctx.expect_gap(
            tower,
            bell,
            axis="y",
            positive_elem="front_left_pier",
            negative_elem="bell_shell",
            min_gap=0.03,
            name="bell clears the left front pier",
        )
        ctx.expect_gap(
            bell,
            tower,
            axis="y",
            positive_elem="bell_shell",
            negative_elem="front_right_pier_mirror",
            min_gap=0.03,
            name="bell clears the right front pier",
        )
        ctx.expect_overlap(
            tower,
            bell,
            axes="y",
            elem_a="headstock_beam",
            elem_b="bell_shell",
            min_overlap=0.55,
            name="bell remains centered under the beam span",
        )

    def elem_center_x(part_name, elem_name):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    rest_pos = elem_center_x("bell", "bell_shell")
    with ctx.pose({swing: limits.upper}):
        upper_pos = elem_center_x("bell", "bell_shell")
    with ctx.pose({swing: limits.lower}):
        lower_pos = elem_center_x("bell", "bell_shell")

    ctx.check(
        "positive bell swing moves toward the opening",
        rest_pos is not None
        and upper_pos is not None
        and upper_pos > rest_pos + 0.18,
        details=f"rest={rest_pos}, upper={upper_pos}",
    )
    ctx.check(
        "negative bell swing retreats inward",
        rest_pos is not None
        and lower_pos is not None
        and lower_pos < rest_pos - 0.18,
        details=f"rest={rest_pos}, lower={lower_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
