from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_air_purifier", assets=ASSETS)

    shell = model.material("shell_white", rgba=(0.93, 0.94, 0.92, 1.0))
    grille = model.material("grille_charcoal", rgba=(0.24, 0.27, 0.29, 1.0))
    accent = model.material("accent_black", rgba=(0.14, 0.16, 0.18, 1.0))
    filter_mat = model.material("filter_gray", rgba=(0.58, 0.60, 0.62, 1.0))

    width = 0.20
    depth = 0.14
    height = 0.18
    wall = 0.008
    top_thickness = 0.010

    inner_width = width - 2.0 * wall
    inner_depth = depth - 2.0 * wall

    hinge_barrel_radius = 0.004
    hinge_pin_radius = 0.0026
    door_thickness = 0.0065
    door_width = inner_width - 0.004
    door_depth = inner_depth - 2.0 * hinge_barrel_radius
    hinge_y = -depth / 2.0 + wall + hinge_barrel_radius

    body = model.part("body")
    wall_height = height - top_thickness
    wall_center_z = wall_height / 2.0

    body.visual(
        Box((width, depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, height - top_thickness / 2.0)),
        material=shell,
        name="top_shell",
    )
    body.visual(
        Box((wall, depth, wall_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, wall_center_z)),
        material=shell,
        name="left_shell",
    )
    body.visual(
        Box((wall, depth, wall_height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, wall_center_z)),
        material=shell,
        name="right_shell",
    )
    body.visual(
        Box((width - 2.0 * wall, wall, wall_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, wall_center_z)),
        material=shell,
        name="front_shell",
    )
    body.visual(
        Box((width - 2.0 * wall, wall, wall_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, wall_center_z)),
        material=shell,
        name="rear_shell",
    )

    body.visual(
        Box((0.15, 0.006, 0.128)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.003, 0.092)),
        material=accent,
        name="front_intake_frame",
    )
    for i in range(7):
        x = -0.054 + i * 0.018
        body.visual(
            Box((0.010, 0.002, 0.116)),
            origin=Origin(xyz=(x, depth / 2.0 + 0.001, 0.092)),
            material=grille,
            name=f"front_slat_{i + 1}",
        )

    body.visual(
        Box((0.060, 0.028, 0.002)),
        origin=Origin(xyz=(0.0, 0.028, height + 0.001)),
        material=accent,
        name="control_pad",
    )
    for i in range(6):
        y = -0.026 + i * 0.0105
        body.visual(
            Box((0.122, 0.005, 0.002)),
            origin=Origin(xyz=(0.0, y, height + 0.001)),
            material=grille,
            name=f"top_outlet_slat_{i + 1}",
        )

    body.visual(
        Box((door_width - 0.016, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, hinge_y - 0.003, -0.004)),
        material=shell,
        name="hinge_cradle",
    )
    body.visual(
        Box((0.022, 0.010, 0.008)),
        origin=Origin(xyz=(-door_width / 2.0 + 0.016, hinge_y - 0.001, -0.004)),
        material=shell,
        name="left_hinge_block",
    )
    body.visual(
        Box((0.022, 0.010, 0.008)),
        origin=Origin(xyz=(door_width / 2.0 - 0.016, hinge_y - 0.001, -0.004)),
        material=shell,
        name="right_hinge_block",
    )
    body.visual(
        Cylinder(radius=hinge_pin_radius, length=door_width - 0.030),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0), xyz=(0.0, hinge_y, -hinge_barrel_radius)),
        material=grille,
        name="hinge_pin",
    )
    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    door = model.part("filter_door")
    door.visual(
        Box((door_width, door_depth, door_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                hinge_barrel_radius + door_depth / 2.0,
                hinge_barrel_radius - door_thickness / 2.0,
            )
        ),
        material=filter_mat,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=hinge_barrel_radius, length=door_width - 0.030),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0), xyz=(0.0, 0.0, 0.0)),
        material=accent,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.072, 0.014, 0.008)),
        origin=Origin(
            xyz=(
                0.0,
                hinge_barrel_radius + door_depth - 0.010,
                hinge_barrel_radius - door_thickness - 0.004,
            )
        ),
        material=accent,
        name="pull_lip",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_depth, door_thickness)),
        mass=0.22,
        origin=Origin(
            xyz=(
                0.0,
                hinge_barrel_radius + door_depth / 2.0,
                hinge_barrel_radius - door_thickness / 2.0,
            )
        ),
    )

    model.articulation(
        "filter_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, hinge_y, -hinge_barrel_radius)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-1.45,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    door = object_model.get_part("filter_door")
    hinge = object_model.get_articulation("filter_door_hinge")
    front_shell = body.get_visual("front_shell")
    rear_shell = body.get_visual("rear_shell")
    top_shell = body.get_visual("top_shell")
    front_intake_frame = body.get_visual("front_intake_frame")
    top_outlet_slat = body.get_visual("top_outlet_slat_1")
    hinge_pin = body.get_visual("hinge_pin")
    door_panel = door.get_visual("door_panel")
    door_barrel = door.get_visual("hinge_barrel")
    pull_lip = door.get_visual("pull_lip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(door, body, reason="Rear filter-door barrel nests around the fixed hinge pin and cradle.")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance(door, body, axes="x", max_dist=0.001)
    ctx.expect_within(door, body, axes="xy", inner_elem=door_panel)
    ctx.expect_gap(
        body,
        door,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=front_shell,
        negative_elem=door_panel,
    )
    ctx.expect_overlap(door, body, axes="xy", min_overlap=0.110, elem_a=door_panel)
    ctx.expect_gap(
        body,
        door,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=rear_shell,
        negative_elem=door_barrel,
    )
    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.005, elem_a=door_barrel, elem_b=hinge_pin)
    ctx.expect_gap(
        body,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=front_intake_frame,
        negative_elem=front_shell,
    )
    ctx.expect_overlap(body, body, axes="xz", min_overlap=0.120, elem_a=front_intake_frame, elem_b=front_shell)
    ctx.expect_gap(
        body,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=top_outlet_slat,
        negative_elem=top_shell,
    )

    with ctx.pose({hinge: -1.20}):
        ctx.expect_gap(
            body,
            door,
            axis="z",
            min_gap=0.035,
            positive_elem=front_shell,
            negative_elem=pull_lip,
        )
        ctx.expect_gap(
            body,
            door,
            axis="z",
            max_gap=0.001,
            max_penetration=1e-6,
            positive_elem=rear_shell,
            negative_elem=door_barrel,
        )
        ctx.expect_origin_distance(door, body, axes="x", max_dist=0.001)
        ctx.expect_overlap(door, body, axes="xz", min_overlap=0.005, elem_a=door_barrel, elem_b=hinge_pin)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
