"""
Base repository pattern for database operations
"""

from typing import Generic, TypeVar

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlmodel import SQLModel

ModelType = TypeVar("ModelType", bound=SQLModel)


class BaseRepository(Generic[ModelType]):
    """
    Base repository providing common CRUD operations

    This implements the Repository pattern to abstract database operations
    and provide a clean interface for data access.
    """

    def __init__(self, model: type[ModelType], db: AsyncSession):
        """
        Initialize repository

        Args:
            model: SQLModel class for this repository
            db: Async database session
        """
        self.model = model
        self.db = db

    async def create(self, obj: ModelType) -> ModelType:
        """
        Create new record

        Args:
            obj: Model instance to create

        Returns:
            Created model instance with ID populated
        """
        self.db.add(obj)
        await self.db.flush()
        await self.db.refresh(obj)
        return obj

    async def get(self, id: int) -> ModelType | None:
        """
        Get record by ID

        Args:
            id: Primary key ID

        Returns:
            Model instance or None if not found
        """
        return await self.db.get(self.model, id)

    async def get_by_field(self, field_name: str, value: any) -> ModelType | None:
        """
        Get single record by field value

        Args:
            field_name: Field name to filter by
            value: Value to match

        Returns:
            First matching model instance or None
        """
        stmt = select(self.model).where(getattr(self.model, field_name) == value)
        result = await self.db.execute(stmt)
        return result.scalar_one_or_none()

    async def list_all(self, limit: int | None = None) -> list[ModelType]:
        """
        List all records

        Args:
            limit: Optional limit on number of records

        Returns:
            List of model instances
        """
        stmt = select(self.model)
        if limit:
            stmt = stmt.limit(limit)
        result = await self.db.execute(stmt)
        return list(result.scalars().all())

    async def update(self, obj: ModelType) -> ModelType:
        """
        Update existing record

        Args:
            obj: Model instance with updated fields

        Returns:
            Updated model instance
        """
        self.db.add(obj)
        await self.db.flush()
        await self.db.refresh(obj)
        return obj

    async def delete(self, obj: ModelType) -> None:
        """
        Delete record

        Args:
            obj: Model instance to delete
        """
        await self.db.delete(obj)
        await self.db.flush()
