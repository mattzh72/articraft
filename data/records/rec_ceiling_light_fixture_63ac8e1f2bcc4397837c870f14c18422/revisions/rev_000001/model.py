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
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_loop(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d, z),
        (half_w, -half_d, z),
        (half_w, half_d, z),
        (-half_w, half_d, z),
    ]


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_carriage_lantern")

    cast_iron = model.material("cast_iron", rgba=(0.17, 0.15, 0.11, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.28, 0.21, 0.13, 1.0))
    glass = model.material("seeded_glass", rgba=(0.83, 0.88, 0.92, 0.32))
    candle = model.material("candle_ivory", rgba=(0.95, 0.92, 0.84, 1.0))
    warm_metal = model.material("warm_metal", rgba=(0.63, 0.51, 0.34, 1.0))

    housing = model.part("housing")

    housing.visual(
        Box((0.186, 0.186, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.564)),
        material=cast_iron,
        name="ceiling_canopy",
    )
    housing.visual(
        Box((0.205, 0.205, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.545)),
        material=bronze,
        name="canopy_rim",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.517)),
        material=cast_iron,
        name="mount_stem",
    )
    housing.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.485)),
        material=bronze,
        name="upper_collar",
    )

    roof_mesh = mesh_from_geometry(
        LoftGeometry(
            [
                _rect_loop(0.300, 0.300, 0.430),
                _rect_loop(0.246, 0.246, 0.452),
                _rect_loop(0.172, 0.172, 0.476),
            ],
            cap=True,
            closed=True,
        ),
        "carriage_lantern_roof",
    )
    housing.visual(
        roof_mesh,
        material=cast_iron,
        name="roof_cap",
    )
    housing.visual(
        Box((0.246, 0.246, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=bronze,
        name="roof_apron",
    )

    frame_height = 0.324
    post_size = 0.026
    frame_outer = 0.230
    frame_half = frame_outer * 0.5
    post_center = frame_half - post_size * 0.5
    rail_span = frame_outer - 2.0 * post_size

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            housing.visual(
                Box((post_size, post_size, frame_height)),
                origin=Origin(xyz=(sx * post_center, sy * post_center, 0.258)),
                material=cast_iron,
                name=f"corner_post_{'r' if sx > 0 else 'l'}_{'f' if sy > 0 else 'b'}",
            )

    housing.visual(
        Box((rail_span, post_size, 0.022)),
        origin=Origin(xyz=(0.0, post_center, 0.107)),
        material=cast_iron,
        name="front_sill",
    )
    housing.visual(
        Box((rail_span, post_size, 0.022)),
        origin=Origin(xyz=(0.0, -post_center, 0.107)),
        material=cast_iron,
        name="rear_sill",
    )
    housing.visual(
        Box((post_size, rail_span, 0.022)),
        origin=Origin(xyz=(post_center, 0.0, 0.107)),
        material=cast_iron,
        name="right_sill",
    )
    housing.visual(
        Box((post_size, rail_span, 0.022)),
        origin=Origin(xyz=(-post_center, 0.0, 0.107)),
        material=cast_iron,
        name="left_sill",
    )

    housing.visual(
        Box((rail_span, post_size, 0.022)),
        origin=Origin(xyz=(0.0, post_center, 0.409)),
        material=cast_iron,
        name="front_lintel",
    )
    housing.visual(
        Box((rail_span, post_size, 0.022)),
        origin=Origin(xyz=(0.0, -post_center, 0.409)),
        material=cast_iron,
        name="rear_lintel",
    )
    housing.visual(
        Box((post_size, rail_span, 0.022)),
        origin=Origin(xyz=(post_center, 0.0, 0.409)),
        material=cast_iron,
        name="right_lintel",
    )
    housing.visual(
        Box((post_size, rail_span, 0.022)),
        origin=Origin(xyz=(-post_center, 0.0, 0.409)),
        material=cast_iron,
        name="left_lintel",
    )

    housing.visual(
        Box((0.220, 0.220, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=bronze,
        name="underside_skirt",
    )
    housing.visual(
        Box((0.196, 0.196, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=cast_iron,
        name="bottom_pan",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=bronze,
        name="lower_finial_stem",
    )
    housing.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=bronze,
        name="lower_finial_ball",
    )

    housing.visual(
        Cylinder(radius=0.012, length=0.266),
        origin=Origin(xyz=(0.0, 0.0, 0.229)),
        material=warm_metal,
        name="candle_support_post",
    )
    housing.visual(
        Cylinder(radius=0.008, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.391)),
        material=warm_metal,
        name="upper_support_post",
    )
    housing.visual(
        Cylinder(radius=0.030, length=0.098),
        origin=Origin(xyz=(0.0, 0.0, 0.181)),
        material=candle,
        name="candle_sleeve",
    )
    housing.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=warm_metal,
        name="candle_cup",
    )

    door_height = 0.276
    door_width = 0.174
    door_depth = 0.016
    frame_band = 0.018
    hinge_plate_thickness = 0.004
    glass_width = door_width - 2.0 * frame_band + 0.002
    glass_height = door_height - 2.0 * frame_band + 0.002
    door_mid_z = 0.258
    front_hinge_y = 0.108
    side_hinge_x = 0.108
    hinge_x = 0.087
    hinge_y = 0.087

    def add_hinge_plate(
        *,
        name: str,
        xyz: tuple[float, float, float],
        yaw: float,
    ) -> None:
        housing.visual(
            Box((0.012, hinge_plate_thickness, door_height)),
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, yaw)),
            material=bronze,
            name=name,
        )

    add_hinge_plate(
        name="front_hinge_plate",
        xyz=(-hinge_x - 0.004, front_hinge_y - 0.012, door_mid_z),
        yaw=0.0,
    )
    add_hinge_plate(
        name="right_hinge_plate",
        xyz=(side_hinge_x - 0.012, hinge_y + 0.004, door_mid_z),
        yaw=-math.pi / 2.0,
    )
    add_hinge_plate(
        name="rear_hinge_plate",
        xyz=(hinge_x + 0.004, -front_hinge_y + 0.012, door_mid_z),
        yaw=math.pi,
    )
    add_hinge_plate(
        name="left_hinge_plate",
        xyz=(-side_hinge_x + 0.012, -hinge_y - 0.004, door_mid_z),
        yaw=math.pi / 2.0,
    )

    housing.inertial = Inertial.from_geometry(
        Box((0.300, 0.300, 0.582)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.291)),
    )

    def add_door(
        *,
        part_name: str,
        joint_name: str,
        hinge_xyz: tuple[float, float, float],
        yaw: float,
        frame_material: Material,
    ) -> None:
        door = model.part(part_name)

        door.visual(
            Box((frame_band, door_depth, door_height)),
            origin=Origin(xyz=(frame_band * 0.5, -0.002, 0.0)),
            material=frame_material,
            name="hinge_stile",
        )
        door.visual(
            Box((frame_band, door_depth, door_height)),
            origin=Origin(xyz=(door_width - frame_band * 0.5, -0.002, 0.0)),
            material=frame_material,
            name="latch_stile",
        )
        door.visual(
            Box((door_width, door_depth, frame_band)),
            origin=Origin(xyz=(door_width * 0.5, -0.002, door_height * 0.5 - frame_band * 0.5)),
            material=frame_material,
            name="top_rail",
        )
        door.visual(
            Box((door_width, door_depth, frame_band)),
            origin=Origin(xyz=(door_width * 0.5, -0.002, -door_height * 0.5 + frame_band * 0.5)),
            material=frame_material,
            name="bottom_rail",
        )
        door.visual(
            Box((glass_width, 0.004, glass_height)),
            origin=Origin(xyz=(door_width * 0.5, -0.001, 0.0)),
            material=glass,
            name="glass_panel",
        )
        for strap_index, strap_z in enumerate((-0.088, 0.0, 0.088)):
            door.visual(
                Box((0.012, 0.004, 0.052)),
                origin=Origin(xyz=(0.006, 0.007, strap_z)),
                material=bronze,
                name=f"hinge_strap_{strap_index}",
            )
        door.visual(
            Box((0.008, 0.004, 0.060)),
            origin=Origin(xyz=(door_width - 0.006, 0.007, 0.0)),
            material=bronze,
            name="pull_handle",
        )
        door.inertial = Inertial.from_geometry(
            Box((door_width, door_depth, door_height)),
            mass=1.1,
            origin=Origin(xyz=(door_width * 0.5, -0.002, 0.0)),
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=housing,
            child=door,
            origin=Origin(xyz=hinge_xyz, rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.2,
                lower=0.0,
                upper=1.45,
            ),
        )

    add_door(
        part_name="front_door",
        joint_name="front_door_hinge",
        hinge_xyz=(-hinge_x, front_hinge_y, door_mid_z),
        yaw=0.0,
        frame_material=cast_iron,
    )
    add_door(
        part_name="right_door",
        joint_name="right_door_hinge",
        hinge_xyz=(side_hinge_x, hinge_y, door_mid_z),
        yaw=-math.pi / 2.0,
        frame_material=cast_iron,
    )
    add_door(
        part_name="rear_door",
        joint_name="rear_door_hinge",
        hinge_xyz=(hinge_x, -front_hinge_y, door_mid_z),
        yaw=math.pi,
        frame_material=cast_iron,
    )
    add_door(
        part_name="left_door",
        joint_name="left_door_hinge",
        hinge_xyz=(-side_hinge_x, -hinge_y, door_mid_z),
        yaw=math.pi / 2.0,
        frame_material=cast_iron,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("housing")
    front_door = object_model.get_part("front_door")
    right_door = object_model.get_part("right_door")
    rear_door = object_model.get_part("rear_door")
    left_door = object_model.get_part("left_door")

    front_hinge = object_model.get_articulation("front_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    rear_hinge = object_model.get_articulation("rear_door_hinge")
    left_hinge = object_model.get_articulation("left_door_hinge")

    ctx.expect_contact(
        front_door,
        housing,
        elem_b="front_hinge_plate",
        contact_tol=0.001,
        name="front door mounts against front hinge plate",
    )
    ctx.expect_contact(
        right_door,
        housing,
        elem_b="right_hinge_plate",
        contact_tol=0.001,
        name="right door mounts against right hinge plate",
    )
    ctx.expect_contact(
        rear_door,
        housing,
        elem_b="rear_hinge_plate",
        contact_tol=0.001,
        name="rear door mounts against rear hinge plate",
    )
    ctx.expect_contact(
        left_door,
        housing,
        elem_b="left_hinge_plate",
        contact_tol=0.001,
        name="left door mounts against left hinge plate",
    )

    def check_outward_swing(
        *,
        label: str,
        door,
        hinge,
        axis_index: int,
        expected_sign: float,
    ) -> None:
        closed_aabb = ctx.part_world_aabb(door)
        with ctx.pose({hinge: 1.1}):
            open_aabb = ctx.part_world_aabb(door)

        closed_center = _aabb_center(closed_aabb)
        open_center = _aabb_center(open_aabb)
        delta = None
        ok = False
        if closed_center is not None and open_center is not None:
            delta = open_center[axis_index] - closed_center[axis_index]
            ok = delta * expected_sign > 0.035
        ctx.check(
            f"{label} swings outward",
            ok,
            details=f"closed_center={closed_center}, open_center={open_center}, delta={delta}",
        )

    check_outward_swing(
        label="front door",
        door=front_door,
        hinge=front_hinge,
        axis_index=1,
        expected_sign=1.0,
    )
    check_outward_swing(
        label="right door",
        door=right_door,
        hinge=right_hinge,
        axis_index=0,
        expected_sign=1.0,
    )
    check_outward_swing(
        label="rear door",
        door=rear_door,
        hinge=rear_hinge,
        axis_index=1,
        expected_sign=-1.0,
    )
    check_outward_swing(
        label="left door",
        door=left_door,
        hinge=left_hinge,
        axis_index=0,
        expected_sign=-1.0,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
