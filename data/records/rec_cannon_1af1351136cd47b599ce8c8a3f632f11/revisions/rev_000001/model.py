from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _wp_from_solid(solid: cq.Solid) -> cq.Workplane:
    return cq.Workplane("XY").add(solid)


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
) -> cq.Workplane:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("Cylinder endpoints must be separated.")
    solid = cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(sx, sy, sz),
        cq.Vector(dx / length, dy / length, dz / length),
    )
    return _wp_from_solid(solid)


def _box(
    center: tuple[float, float, float],
    size: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _carriage_shape() -> cq.Workplane:
    """Connected lightweight carriage frame in cannon coordinates.

    +X points toward the muzzle, +Y is across the axle, and +Z is up.
    """
    shape = _box((-0.09, 0.0, 0.405), (0.36, 0.32, 0.16))

    # Side cheek plates and lower carriage saddle.
    for side in (-1.0, 1.0):
        shape = shape.union(_box((0.04, side * 0.16, 0.55), (0.58, 0.045, 0.25)))
        shape = shape.union(
            _cylinder_between((-0.30, side * 0.10, 0.33), (-0.04, side * 0.13, 0.43), 0.020)
        )
        shape = shape.union(
            _cylinder_between((-0.26, side * 0.09, 0.36), (0.26, side * 0.12, 0.49), 0.018)
        )

    # Axle beam, rear hinge cross tube, and cradle cross tubes.
    shape = shape.union(_cylinder_between((-0.29, -0.17, 0.30), (-0.29, 0.17, 0.30), 0.024))
    shape = shape.union(_cylinder_between((0.00, -0.16, 0.48), (0.00, 0.16, 0.48), 0.020))
    shape = shape.union(_cylinder_between((0.22, -0.15, 0.47), (0.22, 0.15, 0.47), 0.020))

    # Two slim recoil/cradle rails under the barrel.
    for side in (-1.0, 1.0):
        shape = shape.union(_cylinder_between((-0.15, side * 0.075, 0.455), (0.55, side * 0.075, 0.455), 0.017))
        # Upper hinge-pin bridges keep the trail pins visibly tied into the frame
        # without intersecting the moving trail barrels below.
        shape = shape.union(
            _cylinder_between((-0.25, side * 0.13, 0.405), (-0.36, side * 0.13, 0.415), 0.012)
        )

    # Small sight/elevation bracket on one cheek for recognizable artillery detail.
    shape = shape.union(_box((0.25, -0.185, 0.62), (0.055, 0.025, 0.11)))
    shape = shape.union(_cylinder_between((0.25, -0.195, 0.66), (0.25, -0.245, 0.66), 0.010))

    return shape


def _barrel_shape() -> cq.Workplane:
    breech_x = -0.24
    muzzle_x = 0.78
    length = muzzle_x - breech_x

    barrel = _wp_from_solid(
        cq.Solid.makeCone(
            0.090,
            0.055,
            length,
            cq.Vector(breech_x, 0.0, 0.0),
            cq.Vector(1.0, 0.0, 0.0),
        )
    )
    # Reinforced breech and muzzle rings, still part of the same cast bronze tube.
    barrel = barrel.union(_cylinder_between((-0.32, 0.0, 0.0), (-0.21, 0.0, 0.0), 0.105))
    barrel = barrel.union(_cylinder_between((0.68, 0.0, 0.0), (0.80, 0.0, 0.0), 0.064))
    barrel = barrel.union(_cylinder_between((-0.39, 0.0, 0.0), (-0.32, 0.0, 0.0), 0.040))

    # Trunnions are cast as a transverse cylinder centered on the elevation axis.
    barrel = barrel.union(_cylinder_between((0.0, -0.145, 0.0), (0.0, 0.145, 0.0), 0.044))

    # A real visible muzzle bore, not a painted-on disc: cut a short deep recess.
    bore = _cylinder_between((0.625, 0.0, 0.0), (0.815, 0.0, 0.0), 0.026)
    barrel = barrel.cut(bore)
    return barrel


def _trail_shape(side: float) -> cq.Workplane:
    # Local frame origin is the vertical folding hinge pin.
    rear = (-1.08, side * 0.36, -0.15)
    shape = _cylinder_between((0.0, 0.0, -0.055), (0.0, 0.0, 0.060), 0.032)
    shape = shape.union(_cylinder_between((-0.015, 0.0, -0.020), rear, 0.026))
    # A light reinforcing strap above the trail tube.
    shape = shape.union(_cylinder_between((-0.10, side * 0.02, 0.005), (-0.78, side * 0.25, -0.100), 0.012))
    # Ground spade, connected to the trail end and canted slightly by being broad in Y.
    shape = shape.union(_box((-1.12, side * 0.38, -0.17), (0.13, 0.12, 0.06)))
    shape = shape.union(_box((-1.19, side * 0.40, -0.11), (0.035, 0.16, 0.16)))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="light_mountain_pack_cannon")

    bronze = model.material("aged_bronze", rgba=(0.72, 0.46, 0.20, 1.0))
    olive = model.material("olive_painted_steel", rgba=(0.23, 0.30, 0.18, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.04, 0.045, 0.050, 1.0))
    iron = model.material("dark_iron_tire", rgba=(0.08, 0.08, 0.075, 1.0))
    wood = model.material("oiled_walnut", rgba=(0.44, 0.25, 0.12, 1.0))

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_frame", tolerance=0.0015),
        material=olive,
        name="carriage_frame",
    )
    carriage.visual(
        Cylinder(radius=0.028, length=0.780),
        origin=Origin(xyz=(-0.05, 0.0, 0.34), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle",
    )

    # Visible fixed hinge pins for the two folding trail legs.
    for index, side in enumerate((1.0, -1.0)):
        carriage.visual(
            Cylinder(radius=0.015, length=0.300),
            origin=Origin(xyz=(-0.36, side * 0.13, 0.29)),
            material=dark_steel,
            name=f"trail_pin_{index}",
        )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_barrel_shape(), "tapered_bronze_barrel", tolerance=0.0009),
        material=bronze,
        name="barrel_shell",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.290,
            0.075,
            rim=WheelRim(inner_radius=0.205, flange_height=0.014, flange_thickness=0.006),
            hub=WheelHub(
                radius=0.052,
                width=0.090,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.064, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=10, thickness=0.008, window_radius=0.020),
            bore=WheelBore(style="round", diameter=0.032),
        ),
        "wood_spoked_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.325,
            0.090,
            inner_radius=0.285,
            tread=TireTread(style="circumferential", depth=0.0025, count=2),
            grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.002),),
            sidewall=TireSidewall(style="square", bulge=0.015),
        ),
        "iron_wheel_tire",
    )

    wheels = []
    for index in (0, 1):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=wood,
            name="spoked_wheel",
        )
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=iron,
            name="iron_tire",
        )
        wheels.append(wheel)

    trails = []
    for index, side in enumerate((1.0, -1.0)):
        trail = model.part(f"trail_{index}")
        trail.visual(
            mesh_from_cadquery(_trail_shape(side), f"split_trail_{index}", tolerance=0.0015),
            material=olive,
            name="trail_shell",
        )
        trails.append(trail)

    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=-0.16, upper=0.50),
    )

    for index, (wheel, side) in enumerate(zip(wheels, (1.0, -1.0))):
        model.articulation(
            f"carriage_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel,
            origin=Origin(xyz=(-0.05, side * 0.400, 0.34)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=10.0),
        )

    # Both trail legs use positive q to fold inward toward the centerline.
    model.articulation(
        "carriage_to_trail_0",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=trails[0],
        origin=Origin(xyz=(-0.36, 0.13, 0.29)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=0.42),
    )
    model.articulation(
        "carriage_to_trail_1",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=trails[1],
        origin=Origin(xyz=(-0.36, -0.13, 0.29)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    trail_0 = object_model.get_part("trail_0")
    trail_1 = object_model.get_part("trail_1")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    barrel_joint = object_model.get_articulation("carriage_to_barrel")
    trail_joint_0 = object_model.get_articulation("carriage_to_trail_0")
    trail_joint_1 = object_model.get_articulation("carriage_to_trail_1")

    ctx.allow_overlap(
        carriage,
        barrel,
        elem_a="carriage_frame",
        elem_b="barrel_shell",
        reason="The bronze trunnions are intentionally seated in the carriage cheek bearings.",
    )
    ctx.expect_overlap(
        barrel,
        carriage,
        axes="y",
        elem_a="barrel_shell",
        elem_b="carriage_frame",
        min_overlap=0.28,
        name="trunnions_span_between_cheek_bearings",
    )

    for index, wheel in enumerate((wheel_0, wheel_1)):
        ctx.allow_overlap(
            carriage,
            wheel,
            elem_a="axle",
            elem_b="spoked_wheel",
            reason="The wheel hub is simplified as a solid spoked wheel captured on the fixed axle stub.",
        )
        ctx.expect_overlap(
            carriage,
            wheel,
            axes="y",
            elem_a="axle",
            elem_b="spoked_wheel",
            min_overlap=0.020,
            name=f"wheel_{index}_hub_captured_on_axle",
        )
        ctx.expect_within(
            carriage,
            wheel,
            axes="xz",
            inner_elem="axle",
            outer_elem="spoked_wheel",
            margin=0.002,
            name=f"wheel_{index}_axle_centered_in_hub",
        )

    for index, trail in enumerate((trail_0, trail_1)):
        ctx.allow_overlap(
            carriage,
            trail,
            elem_a=f"trail_pin_{index}",
            elem_b="trail_shell",
            reason="The fixed hinge pin is intentionally captured inside the simplified solid trail hinge barrel.",
        )
        ctx.expect_within(
            carriage,
            trail,
            axes="xy",
            inner_elem=f"trail_pin_{index}",
            outer_elem="trail_shell",
            margin=0.001,
            name=f"trail_{index}_pin_lies_inside_hinge_barrel",
        )
        ctx.expect_overlap(
            carriage,
            trail,
            axes="z",
            elem_a=f"trail_pin_{index}",
            elem_b="trail_shell",
            min_overlap=0.10,
            name=f"trail_{index}_pin_retained_vertically",
        )

    ctx.check("has_two_spoked_wheels", wheel_0 is not None and wheel_1 is not None)
    ctx.check("has_two_split_trails", trail_0 is not None and trail_1 is not None)
    ctx.check(
        "primary_revolute_joints_present",
        barrel_joint is not None and trail_joint_0 is not None and trail_joint_1 is not None,
    )

    barrel_aabb = ctx.part_world_aabb(barrel)
    ctx.check("short_barrel_aabb_present", barrel_aabb is not None)
    if barrel_aabb is not None:
        mins, maxs = barrel_aabb
        length = float(maxs[0] - mins[0])
        height = float(maxs[2] - mins[2])
        ctx.check("short_tapered_barrel_scale", 0.95 <= length <= 1.25 and 0.16 <= height <= 0.24)

    wheel_aabb = ctx.part_world_aabb(wheel_0)
    if wheel_aabb is not None:
        mins, maxs = wheel_aabb
        size = sorted(float(maxs[i] - mins[i]) for i in range(3))
        wheel_width = size[0]
        wheel_diameter = size[2]
        ctx.check("pack_cannon_wheel_scale", 0.60 <= wheel_diameter <= 0.70 and 0.07 <= wheel_width <= 0.13)

    rest_barrel = ctx.part_world_aabb(barrel)
    with ctx.pose({barrel_joint: 0.45}):
        raised_barrel = ctx.part_world_aabb(barrel)
    if rest_barrel is not None and raised_barrel is not None:
        ctx.check(
            "positive_barrel_elevation_raises_muzzle",
            float(raised_barrel[1][2]) > float(rest_barrel[1][2]) + 0.10,
        )

    def _center_y(part):
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return 0.5 * (float(aabb[0][1]) + float(aabb[1][1]))

    rest_y_0 = _center_y(trail_0)
    rest_y_1 = _center_y(trail_1)
    with ctx.pose({trail_joint_0: 0.42, trail_joint_1: 0.42}):
        folded_y_0 = _center_y(trail_0)
        folded_y_1 = _center_y(trail_1)
    ctx.check(
        "trail_0_folds_inward",
        rest_y_0 is not None and folded_y_0 is not None and folded_y_0 < rest_y_0 - 0.10,
    )
    ctx.check(
        "trail_1_folds_inward",
        rest_y_1 is not None and folded_y_1 is not None and folded_y_1 > rest_y_1 + 0.10,
    )

    return ctx.report()


object_model = build_object_model()
