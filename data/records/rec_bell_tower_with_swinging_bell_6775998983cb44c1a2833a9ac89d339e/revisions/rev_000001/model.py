from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="english_parish_bell_tower")

    stone = model.material("weathered_limestone", rgba=(0.58, 0.56, 0.50, 1.0))
    dark_stone = model.material("dressed_edge_stone", rgba=(0.47, 0.46, 0.41, 1.0))
    timber = model.material("aged_oak", rgba=(0.37, 0.22, 0.10, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.74, 0.45, 0.18, 1.0))
    iron = model.material("blackened_iron", rgba=(0.04, 0.04, 0.04, 1.0))
    rope_mat = model.material("hemp_rope", rgba=(0.74, 0.62, 0.42, 1.0))
    sally_mat = model.material("wool_sally", rgba=(0.72, 0.08, 0.05, 1.0))

    tower = model.part("stone_tower")

    # Square tower shell: 4 m outside, hollow belfry chamber, and tall openings on all sides.
    outer = 4.00
    wall = 0.45
    half = outer / 2.0
    wall_center = half - wall / 2.0
    body_h = 10.70
    opening_w = 1.50
    opening_bottom = 6.60
    opening_top = 9.60
    jamb_w = (outer - opening_w) / 2.0

    def add_y_wall(prefix: str, y: float) -> None:
        sign = 1.0 if y > 0 else -1.0
        tower.visual(
            Box((jamb_w, wall, body_h)),
            origin=Origin(xyz=(-(opening_w / 2.0 + jamb_w / 2.0), y, body_h / 2.0)),
            material=stone,
            name=f"{prefix}_jamb_0",
        )
        tower.visual(
            Box((jamb_w, wall, body_h)),
            origin=Origin(xyz=((opening_w / 2.0 + jamb_w / 2.0), y, body_h / 2.0)),
            material=stone,
            name=f"{prefix}_jamb_1",
        )
        tower.visual(
            Box((opening_w, wall, opening_bottom)),
            origin=Origin(xyz=(0.0, y, opening_bottom / 2.0)),
            material=stone,
            name=f"{prefix}_spandrel",
        )
        top_h = body_h - opening_top
        tower.visual(
            Box((opening_w, wall, top_h)),
            origin=Origin(xyz=(0.0, y, opening_top + top_h / 2.0)),
            material=stone,
            name=f"{prefix}_arch_mass",
        )
        # Dark angled louvres sit in the belfry opening and are embedded into the jambs.
        for idx, z in enumerate((7.15, 7.75, 8.35, 8.95)):
            tower.visual(
                Box((opening_w + 0.12, 0.08, 0.12)),
                origin=Origin(xyz=(0.0, y - sign * 0.02, z), rpy=(sign * 0.18, 0.0, 0.0)),
                material=timber,
                name=f"{prefix}_louvre_{idx}",
            )

    def add_x_wall(prefix: str, x: float) -> None:
        sign = 1.0 if x > 0 else -1.0
        tower.visual(
            Box((wall, jamb_w, body_h)),
            origin=Origin(xyz=(x, -(opening_w / 2.0 + jamb_w / 2.0), body_h / 2.0)),
            material=stone,
            name=f"{prefix}_jamb_0",
        )
        tower.visual(
            Box((wall, jamb_w, body_h)),
            origin=Origin(xyz=(x, (opening_w / 2.0 + jamb_w / 2.0), body_h / 2.0)),
            material=stone,
            name=f"{prefix}_jamb_1",
        )
        tower.visual(
            Box((wall, opening_w, opening_bottom)),
            origin=Origin(xyz=(x, 0.0, opening_bottom / 2.0)),
            material=stone,
            name=f"{prefix}_spandrel",
        )
        top_h = body_h - opening_top
        tower.visual(
            Box((wall, opening_w, top_h)),
            origin=Origin(xyz=(x, 0.0, opening_top + top_h / 2.0)),
            material=stone,
            name=f"{prefix}_arch_mass",
        )
        for idx, z in enumerate((7.15, 7.75, 8.35, 8.95)):
            tower.visual(
                Box((0.08, opening_w + 0.12, 0.12)),
                origin=Origin(xyz=(x - sign * 0.02, 0.0, z), rpy=(0.0, -sign * 0.18, 0.0)),
                material=timber,
                name=f"{prefix}_louvre_{idx}",
            )

    add_y_wall("front_belfry", wall_center)
    add_y_wall("rear_belfry", -wall_center)
    add_x_wall("side_belfry_0", wall_center)
    add_x_wall("side_belfry_1", -wall_center)

    tower.visual(Box((4.35, 4.35, 0.35)), origin=Origin(xyz=(0.0, 0.0, 0.175)), material=dark_stone, name="plinth")
    for idx, y in enumerate((-2.02, 2.02)):
        tower.visual(Box((4.32, 0.20, 0.16)), origin=Origin(xyz=(0.0, y, 6.52)), material=dark_stone, name=f"string_course_y_{idx}")
        tower.visual(Box((4.36, 0.22, 0.18)), origin=Origin(xyz=(0.0, y, 10.75)), material=dark_stone, name=f"parapet_cornice_y_{idx}")
    for idx, x in enumerate((-2.02, 2.02)):
        tower.visual(Box((0.20, 4.32, 0.16)), origin=Origin(xyz=(x, 0.0, 6.52)), material=dark_stone, name=f"string_course_x_{idx}")
        tower.visual(Box((0.22, 4.36, 0.18)), origin=Origin(xyz=(x, 0.0, 10.75)), material=dark_stone, name=f"parapet_cornice_x_{idx}")
    for ix, x in enumerate((-1.96, 1.96)):
        for iy, y in enumerate((-1.96, 1.96)):
            tower.visual(
                Box((0.30, 0.30, body_h)),
                origin=Origin(xyz=(x, y, body_h / 2.0)),
                material=dark_stone,
                name=f"corner_quoin_{ix}_{iy}",
            )

    # Crenellated parapet ring and merlons.
    parapet_z = 11.08
    tower.visual(Box((outer, wall, 0.62)), origin=Origin(xyz=(0.0, wall_center, parapet_z)), material=stone, name="front_parapet")
    tower.visual(Box((outer, wall, 0.62)), origin=Origin(xyz=(0.0, -wall_center, parapet_z)), material=stone, name="rear_parapet")
    tower.visual(Box((wall, outer, 0.62)), origin=Origin(xyz=(wall_center, 0.0, parapet_z)), material=stone, name="side_parapet_0")
    tower.visual(Box((wall, outer, 0.62)), origin=Origin(xyz=(-wall_center, 0.0, parapet_z)), material=stone, name="side_parapet_1")
    for idx, pos in enumerate((-1.42, -0.47, 0.47, 1.42)):
        tower.visual(Box((0.50, 0.56, 0.80)), origin=Origin(xyz=(pos, wall_center, 11.79)), material=stone, name=f"front_merlon_{idx}")
        tower.visual(Box((0.50, 0.56, 0.80)), origin=Origin(xyz=(pos, -wall_center, 11.79)), material=stone, name=f"rear_merlon_{idx}")
        tower.visual(Box((0.56, 0.50, 0.80)), origin=Origin(xyz=(wall_center, pos, 11.79)), material=stone, name=f"side_merlon_0_{idx}")
        tower.visual(Box((0.56, 0.50, 0.80)), origin=Origin(xyz=(-wall_center, pos, 11.79)), material=stone, name=f"side_merlon_1_{idx}")

    frame = model.part("belfry_frame")
    for ix, x in enumerate((-1.18, 1.18)):
        for iy, y in enumerate((-1.18, 1.18)):
            frame.visual(
                Box((0.18, 0.18, 2.80)),
                origin=Origin(xyz=(x, y, 8.10)),
                material=timber,
                name=f"post_{ix}_{iy}",
            )
    for idx, y in enumerate((-1.18, 1.18)):
        frame.visual(Box((3.10, 0.16, 0.16)), origin=Origin(xyz=(0.0, y, 6.78)), material=timber, name=f"lower_tie_x_{idx}")
        frame.visual(Box((3.10, 0.16, 0.16)), origin=Origin(xyz=(0.0, y, 9.42)), material=timber, name=f"upper_tie_x_{idx}")
    for idx, x in enumerate((-1.18, 1.18)):
        frame.visual(Box((0.16, 3.10, 0.16)), origin=Origin(xyz=(x, 0.0, 6.96)), material=timber, name=f"lower_tie_y_{idx}")
        frame.visual(Box((0.16, 3.10, 0.16)), origin=Origin(xyz=(x, 0.0, 9.24)), material=timber, name=f"upper_tie_y_{idx}")
    for idx, x in enumerate((-1.12, 1.12)):
        frame.visual(Box((0.18, 2.36, 0.12)), origin=Origin(xyz=(x, 0.0, 8.28)), material=timber, name=f"bearing_rail_{idx}")
    for idx, x in enumerate((-0.95, 0.95)):
        frame.visual(Box((0.36, 0.34, 0.16)), origin=Origin(xyz=(x, 0.0, 8.34)), material=timber, name=f"bearing_block_{idx}")
    model.articulation("tower_to_frame", ArticulationType.FIXED, parent=tower, child=frame, origin=Origin())

    bell = model.part("bell")
    bell.visual(
        Cylinder(radius=0.080, length=2.10),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="swing_axle",
    )
    bell.visual(Box((1.44, 0.36, 0.32)), origin=Origin(xyz=(0.0, 0.0, 0.18)), material=timber, name="headstock")
    bell.visual(Box((0.18, 0.50, 0.48)), origin=Origin(xyz=(-0.54, 0.0, -0.08)), material=timber, name="yoke_cheek_0")
    bell.visual(Box((0.18, 0.50, 0.48)), origin=Origin(xyz=(0.54, 0.0, -0.08)), material=timber, name="yoke_cheek_1")
    bell.visual(Box((0.08, 0.06, 0.70)), origin=Origin(xyz=(-0.34, -0.21, -0.25)), material=iron, name="strap_0")
    bell.visual(Box((0.08, 0.06, 0.70)), origin=Origin(xyz=(0.34, -0.21, -0.25)), material=iron, name="strap_1")
    bell.visual(Box((0.08, 0.06, 0.70)), origin=Origin(xyz=(-0.34, 0.21, -0.25)), material=iron, name="strap_2")
    bell.visual(Box((0.08, 0.06, 0.70)), origin=Origin(xyz=(0.34, 0.21, -0.25)), material=iron, name="strap_3")
    bell.visual(
        Cylinder(radius=0.035, length=0.72),
        origin=Origin(xyz=(0.99, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="wheel_pin",
    )
    bell.visual(
        Cylinder(radius=0.090, length=0.030),
        origin=Origin(xyz=(1.365, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="pin_collar",
    )

    bell_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.20, -0.43),
            (0.28, -0.62),
            (0.40, -0.94),
            (0.54, -1.34),
            (0.68, -1.68),
            (0.82, -1.93),
        ],
        inner_profile=[
            (0.10, -0.52),
            (0.18, -0.73),
            (0.32, -1.03),
            (0.45, -1.38),
            (0.58, -1.66),
            (0.69, -1.84),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    bell.visual(mesh_from_geometry(bell_shell, "bronze_bell_shell"), material=bronze, name="bell_shell")
    bell.visual(Cylinder(radius=0.17, length=0.13), origin=Origin(xyz=(0.0, 0.0, -0.43)), material=bronze, name="crown_cap")
    bell.visual(Box((0.18, 0.18, 0.58)), origin=Origin(xyz=(0.0, 0.0, -0.15)), material=bronze, name="crown_boss")
    bell.visual(Cylinder(radius=0.025, length=1.25), origin=Origin(xyz=(0.0, 0.0, -1.05)), material=iron, name="clapper_stem")
    bell.visual(Sphere(radius=0.12), origin=Origin(xyz=(0.0, 0.0, -1.72)), material=iron, name="clapper_ball")

    model.articulation(
        "bell_axle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 8.50)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.4, lower=-1.20, upper=1.20),
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.65,
                0.10,
                rim=WheelRim(inner_radius=0.53, flange_height=0.025, flange_thickness=0.012, bead_seat_depth=0.005),
                hub=WheelHub(radius=0.12, width=0.10, cap_style="flat"),
                face=WheelFace(dish_depth=0.0, front_inset=0.004, rear_inset=0.004),
                spokes=WheelSpokes(style="straight", count=8, thickness=0.035, window_radius=0.035),
                bore=WheelBore(style="round", diameter=0.090),
            ),
            "wooden_ringing_wheel",
        ),
        material=timber,
        name="ringing_wheel",
    )
    wheel.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, -0.62, -0.18),
                    (0.0, -0.68, -0.55),
                    (0.0, -0.68, -1.45),
                    (0.0, -0.68, -3.40),
                    (0.0, -0.62, -4.70),
                ],
                radius=0.022,
                samples_per_segment=8,
                radial_segments=14,
            ),
            "bell_rope",
        ),
        material=rope_mat,
        name="pull_rope",
    )
    wheel.visual(Cylinder(radius=0.055, length=0.46), origin=Origin(xyz=(0.0, -0.68, -3.15)), material=sally_mat, name="rope_sally")
    model.articulation(
        "wheel_pin",
        ArticulationType.REVOLUTE,
        parent=bell,
        child=wheel,
        origin=Origin(xyz=(1.30, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("stone_tower")
    frame = object_model.get_part("belfry_frame")
    bell = object_model.get_part("bell")
    wheel = object_model.get_part("wheel")
    bell_joint = object_model.get_articulation("bell_axle")
    wheel_joint = object_model.get_articulation("wheel_pin")

    ctx.expect_gap(
        bell,
        frame,
        axis="z",
        positive_elem="swing_axle",
        negative_elem="bearing_block_1",
        max_gap=0.002,
        max_penetration=0.0,
        name="bell axle rests on timber bearing block",
    )
    ctx.expect_overlap(
        bell,
        frame,
        axes="xy",
        elem_a="swing_axle",
        elem_b="bearing_block_1",
        min_overlap=0.12,
        name="bearing block is under the horizontal axle",
    )
    ctx.expect_within(wheel, tower, axes="xy", margin=0.0, name="ringing wheel remains inside tower plan")
    ctx.expect_within(bell, tower, axes="xy", margin=0.0, name="bronze bell hangs within square tower")

    def elem_center(part_name: str, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(object_model.get_part(part_name), elem=elem)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple(float((mins[i] + maxs[i]) * 0.5) for i in range(3))

    bell_rest = elem_center("bell", "bell_shell")
    with ctx.pose({bell_joint: 0.70}):
        bell_swung = elem_center("bell", "bell_shell")
    ctx.check(
        "bell swings about horizontal axle",
        bell_rest is not None
        and bell_swung is not None
        and bell_swung[1] > bell_rest[1] + 0.35
        and bell_swung[2] > bell_rest[2] + 0.12,
        details=f"rest={bell_rest}, swung={bell_swung}",
    )

    rope_rest = elem_center("wheel", "pull_rope")
    with ctx.pose({wheel_joint: 0.25}):
        rope_pulled = elem_center("wheel", "pull_rope")
    ctx.check(
        "wheel pin rotates rope pull",
        rope_rest is not None and rope_pulled is not None and abs(rope_pulled[1] - rope_rest[1]) > 0.25,
        details=f"rest={rope_rest}, pulled={rope_pulled}",
    )

    return ctx.report()


object_model = build_object_model()
